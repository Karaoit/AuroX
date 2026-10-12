// AuroX V0.6 - Real local filesystem snapshot, verification and failover.
// Uses ONLY ParameterSpace, StateSpace and ActionSpace (C++20).
//
// No arguments: create sample files in a new temporary directory, inject one
// fast-copy failure, then recover via chunked copy. Leaves a real release.
// Usage with user files: aurox_snapshot SOURCE_DIR OUTPUT_DIR [--fail-fast-once]
// Source and output must be separate, non-nested directories.
// The program never deletes the source, and only cleans up its OWN staging dir.

#include "AuroX/Space/ParameterSpace.hpp"
#include "AuroX/Space/StateSpace.hpp"
#include "AuroX/Space/ActionSpace.hpp"
#include "AuroX/Planner/Planner.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using namespace AuroX::Space;
using AuroX::Planner::bfs;
using AuroX::Planner::Chain;

namespace {

// ---------- 1. ParameterSpace: backup constraints, NOT task progress. ------
struct BackupPolicy : PStruct<BackupPolicy> {
    Param<int> maxFiles, maxFileKiB, maxTotalMiB, ioBufferKiB;
    AUROX_PARAMS(maxFiles, maxFileKiB, maxTotalMiB, ioBufferKiB)

    BackupPolicy() {
        PARAM_REG(
            PARAM_BIND(maxFiles,    100, 1, 10000);
            PARAM_BIND(maxFileKiB, 2048, 1, 65536);
            PARAM_BIND(maxTotalMiB,  64, 1, 1024);
            PARAM_BIND(ioBufferKiB,  64, 1, 1024);
        );
    }
};

struct Rules {
    std::uint64_t maxFiles;
    std::uint64_t maxFileBytes;
    std::uint64_t maxTotalBytes;
    std::size_t bufferBytes;
};

Rules rulesOf(const BackupPolicy& p) {
    if (!p.validate()) throw std::invalid_argument("invalid backup policy");
    constexpr std::uint64_t KiB = 1024;
    constexpr std::uint64_t MiB = 1024 * KiB;
    return {static_cast<std::uint64_t>(p.maxFiles.get()),
            static_cast<std::uint64_t>(p.maxFileKiB.get()) * KiB,
            static_cast<std::uint64_t>(p.maxTotalMiB.get()) * MiB,
            static_cast<std::size_t>(p.ioBufferKiB.get()) * KiB};
}

// ---------- 2. StateSpace: verified current task facts. -------------------
struct BackupState : SSp {
    SField<bool> scanned, validated, prepared, safeReady;
    SField<bool> copied, verified, published;
    SField<int> fileCount;
    SField<std::int64_t> totalBytes;
    SField<Str> note, releasePath;

    BackupState() {
        // These 7 flags form the planning vector, in a frozen layout.
        bind(scanned,    "scanned",    false, "Source inventory created", true);
        bind(validated,  "validated",  false, "Policy and data checked", true);
        bind(prepared,   "prepared",   false, "Staging directory prepared", true);
        bind(safeReady,  "safeReady",  false, "Slow-copy fallback selected", true);
        bind(copied,     "copied",     false, "All files copied", true);
        bind(verified,   "verified",   false, "All content verified", true);
        bind(published,  "published",  false, "Version made visible", true);

        // Operational metadata: name-addressable, but not part of BFS state.
        bind(fileCount,   "fileCount",   0, "Number of input files", false);
        bind(totalBytes,  "totalBytes",  std::int64_t{0}, "Input bytes", false);
        bind(note,        "note",        Str{"ready"}, "Last task detail");
        bind(releasePath, "releasePath", Str{}, "Published snapshot path");
        freeze();
    }
};

struct Indices {
    std::size_t scanned, validated, prepared, safeReady;
    std::size_t copied, verified, published;
};

Indices indicesOf(const BackupState& state) {
    auto locate = [&](const Str& name) {
        for (const auto& entry : state.layout()) {
            if (entry.name == name && entry.components == 1) return entry.offset;
        }
        throw std::logic_error("missing state in vector layout: " + name);
    };
    if (state.dim() != 7) throw std::logic_error("unexpected planning dimension");
    return {locate("scanned"), locate("validated"), locate("prepared"),
            locate("safeReady"), locate("copied"), locate("verified"),
            locate("published")};
}

bool yes(const Vec& x, std::size_t i) { return x.at(i) > 0.5; }

// Task-side error comparison: the Space layer no longer exposes a distance
// function, so the caller compares its own predicted vs. measured vectors
// (squared error; a non-zero difference means the real result diverged).
double sqDist(const Vec& a, const Vec& b) {
    double d = 0.0;
    const std::size_t n = a.size() < b.size() ? a.size() : b.size();
    for (std::size_t i = 0; i < n; ++i) {
        const double delta = a[i] - b[i];
        d += delta * delta;
    }
    return d;
}

std::function<Vec(const Vec&)> change(
    std::initializer_list<std::pair<std::size_t, double>> mutations) {
    std::vector<std::pair<std::size_t, double>> edits(mutations);
    return [edits = std::move(edits)](const Vec& in) {
        Vec out = in;
        for (const auto& [index, value] : edits) out.at(index) = value;
        return out;
    };
}

// ---------- Real filesystem logic. No mocked file operations. ------------
struct FileInfo {
    fs::path relative;
    std::uint64_t bytes = 0;
    std::uint64_t fingerprint = 0;
};

bool nestedWithin(const fs::path& candidate, const fs::path& parent) {
    auto a = candidate.begin();
    auto b = parent.begin();
    for (; b != parent.end(); ++a, ++b) {
        if (a == candidate.end() || *a != *b) return false;
    }
    return true;
}

void writeText(const fs::path& path, const Str& content) {
    fs::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out || !(out << content))
        throw std::runtime_error("cannot write: " + path.string());
}

fs::path uniquePath(const fs::path& parent, const Str& prefix) {
    for (int i = 0; i < 100; ++i) {
        const auto tick = std::chrono::high_resolution_clock::now()
                              .time_since_epoch().count();
        const fs::path candidate = parent / (prefix + std::to_string(tick) +
                                              "_" + std::to_string(i));
        if (!fs::exists(candidate)) return candidate;
    }
    throw std::runtime_error("cannot reserve unique working directory");
}

void createDemoSource(const fs::path& source) {
    writeText(source / "docs" / "intro.txt",
              "AuroX V0.6\nLocal filesystem snapshot demonstration.\n");
    writeText(source / "config" / "app.ini",
              "[server]\nport=8080\nmode=offline\n");
    writeText(source / "data" / "orders.csv",
              "order_id,amount\nA001,35.80\nA002,19.40\nA003,7.50\n");
    writeText(source / "src" / "hello.cpp",
              "#include <iostream>\nint main(){ std::cout << \"hello\"; }\n");
}

// FNV-1a is a deterministic *non-cryptographic* fingerprint, used here to
// catch accidental content changes; this is not a security-grade signature.
std::uint64_t fingerprint(const fs::path& path, std::size_t bufferBytes) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot read: " + path.string());
    std::vector<char> buffer(bufferBytes);
    std::uint64_t hash = 14695981039346656037ULL;
    while (input) {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const auto count = input.gcount();
        for (std::streamsize i = 0; i < count; ++i) {
            hash ^= static_cast<unsigned char>(buffer[static_cast<std::size_t>(i)]);
            hash *= 1099511628211ULL;
        }
    }
    if (!input.eof()) throw std::runtime_error("read failed: " + path.string());
    return hash;
}

bool equalFileBytes(const fs::path& lhs, const fs::path& rhs,
                    std::size_t bufferBytes) {
    std::ifstream a(lhs, std::ios::binary), b(rhs, std::ios::binary);
    if (!a || !b) throw std::runtime_error("cannot open files for comparison");
    std::vector<char> x(bufferBytes), y(bufferBytes);
    for (;;) {
        a.read(x.data(), static_cast<std::streamsize>(x.size()));
        b.read(y.data(), static_cast<std::streamsize>(y.size()));
        const auto n = a.gcount(), m = b.gcount();
        if (n != m || !std::equal(x.begin(), x.begin() + n, y.begin())) return false;
        if (n == 0) {
            if (!a.eof() || !b.eof())
                throw std::runtime_error("file comparison I/O failure");
            return true;
        }
    }
}

struct SnapshotJob {
    fs::path source, output, staging, release;
    Rules rules;
    std::vector<FileInfo> files;
    bool injectFastFailure = false;
    Str lastError;

    SnapshotJob(const fs::path& src, const fs::path& out, Rules r, bool inject)
        : rules(r), injectFastFailure(inject) {
        if (!fs::is_directory(src))
            throw std::invalid_argument("source must be an existing directory");
        source = fs::weakly_canonical(src);
        output = fs::weakly_canonical(out);
        if (nestedWithin(output, source) || nestedWithin(source, output))
            throw std::invalid_argument("source and output cannot contain each other");
        fs::create_directories(output / ".staging");
        fs::create_directories(output / "releases");
        const fs::path placeholder = uniquePath(output / ".staging", "snapshot_");
        staging = placeholder;
        release = output / "releases" / placeholder.filename();
        if (fs::exists(release))
            throw std::runtime_error("release path already exists");
    }

    ~SnapshotJob() {
        // Remove only our own, uniquely named uncommitted staging directory.
        std::error_code ignored;
        fs::remove_all(staging, ignored);
    }

    void scan() {
        files.clear();
        for (const auto& entry : fs::recursive_directory_iterator(source)) {
            if (entry.is_symlink())
                throw std::runtime_error("symlinks not supported: " + entry.path().string());
            if (entry.is_directory()) continue;
            if (!entry.is_regular_file())
                throw std::runtime_error("special file not supported: " + entry.path().string());
            const fs::path relative = entry.path().lexically_relative(source);
            const Str printable = relative.generic_string();
            if (printable.find_first_of("\t\r\n") != Str::npos)
                throw std::runtime_error("filename contains tab/newline: " + printable);
            files.push_back(FileInfo{relative, entry.file_size(), 0});
        }
        std::sort(files.begin(), files.end(), [](const FileInfo& a, const FileInfo& b) {
            return a.relative.generic_string() < b.relative.generic_string();
        });
        if (files.empty()) throw std::runtime_error("source contains no files");
    }

    std::uint64_t totalBytes() const {
        std::uint64_t total = 0;
        for (const auto& f : files) total += f.bytes;
        return total;
    }

    void validate() {
        if (files.size() > rules.maxFiles) throw std::runtime_error("file count limit exceeded");
        std::uint64_t total = 0;
        for (auto& f : files) {
            // The file may have changed between scan and validation.
            const fs::path original = source / f.relative;
            if (!fs::is_regular_file(fs::symlink_status(original)))
                throw std::runtime_error("input type changed: " + original.string());
            f.bytes = fs::file_size(original);
            if (f.bytes > rules.maxFileBytes)
                throw std::runtime_error("per-file limit exceeded: " + original.string());
            if (f.bytes > rules.maxTotalBytes - total)
                throw std::runtime_error("total input size limit exceeded");
            total += f.bytes;
            f.fingerprint = fingerprint(original, rules.bufferBytes);
        }
    }

    void prepare() { fs::create_directories(staging / "data"); }

    void copyFast() {
        std::size_t completed = 0;
        for (const auto& f : files) {
            const auto dest = staging / "data" / f.relative;
            fs::create_directories(dest.parent_path());
            fs::copy_file(source / f.relative, dest, fs::copy_options::overwrite_existing);
            ++completed;
            if (injectFastFailure && completed == 1) {
                injectFastFailure = false; // a one-time simulated failure, in REAL staging I/O
                throw std::runtime_error("injected I/O interruption after first copied file");
            }
        }
    }

    void selectSlowCopy() {
        // Discard only incomplete files in this job's private staging folder.
        fs::remove_all(staging / "data");
        fs::create_directories(staging / "data");
    }

    void copySlow() {
        std::vector<char> buffer(rules.bufferBytes);
        for (const auto& f : files) {
            const auto dest = staging / "data" / f.relative;
            const auto temporary = fs::path(dest.string() + ".part");
            fs::create_directories(dest.parent_path());
            std::ifstream input(source / f.relative, std::ios::binary);
            std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
            if (!input || !out) throw std::runtime_error("cannot open slow-copy streams");
            while (input) {
                input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
                const auto n = input.gcount();
                if (n != 0) out.write(buffer.data(), n);
                if (!out) throw std::runtime_error("slow-copy write failed");
            }
            if (!input.eof()) throw std::runtime_error("slow-copy read failed");
            out.close();
            if (!out) throw std::runtime_error("slow-copy flush failed");
            fs::rename(temporary, dest);
        }
    }

    void verifyAndManifest() {
        const auto manifest = staging / "manifest.tsv";
        std::ofstream report(manifest, std::ios::binary | std::ios::trunc);
        if (!report) throw std::runtime_error("cannot create snapshot manifest");
        report << "path\tbytes\tfnv1a64\n";
        for (const auto& f : files) {
            const auto original = source / f.relative;
            const auto copy = staging / "data" / f.relative;
            if (!fs::is_regular_file(copy) || fs::file_size(copy) != f.bytes ||
                fingerprint(copy, rules.bufferBytes) != f.fingerprint ||
                !equalFileBytes(original, copy, rules.bufferBytes))
                throw std::runtime_error("verification failed: " + f.relative.string());
            report << f.relative.generic_string() << '\t' << f.bytes << '\t'
                   << std::hex << std::setw(16) << std::setfill('0')
                   << f.fingerprint << std::dec << '\n';
        }
        report.close();
        if (!report) throw std::runtime_error("manifest write failed");
    }

    void publish() {
        if (fs::exists(release)) throw std::runtime_error("release name collision");
        // Same output filesystem: directory rename is normally atomic.
        // Existing releases are NEVER overwritten or deleted.
        fs::rename(staging, release);
    }
};

// ---------- 3. ActionSpace: guarded execution + actual I/O. ---------------
struct BackupActions : ASpace {
    Act scan, validate, prepare, fastCopy, selectSlow, slowCopy, verify, publish;

    BackupActions(const Indices& k, SnapshotJob& job, BackupState& state) {
        const auto registerTask = [&](Act& action, Str name,
                                      std::function<bool(const Vec&)> guard,
                                      std::function<Vec(const Vec&)> predict,
                                      std::function<void()> io) {
            action.name = std::move(name);
            action.type = "filesystem-snapshot";
            action.conditions.push_back(Condition{"task prerequisites", std::move(guard)});
            action.nxt = std::move(predict); // PURE, called many times by BFS.
            const Str actionName = action.name;
            action.run = [&job, &state, io = std::move(io), actionName] {
                try {
                    io(); // Actual filesystem work is done ONLY here.
                    job.lastError.clear();
                    state.note = actionName + " OK";
                } catch (const std::exception& e) {
                    // Keep the REAL StateSpace unchanged on failure: run()
                    // returning true only means this closure returned, so the
                    // caller detects the divergence by comparing the predicted
                    // successor against the measured state.
                    job.lastError = e.what();
                    state.note = actionName + " FAILED: " + job.lastError;
                    std::cout << "  [I/O error] " << job.lastError << '\n';
                }
            };
        };

        registerTask(scan, "scan_source",
            [k](const Vec& x) { return !yes(x, k.scanned); },
            change({{k.scanned, 1}}),
            [&] {
                job.scan();
                state.fileCount = static_cast<int>(job.files.size()); // typed write
                state.set("totalBytes", static_cast<std::int64_t>(job.totalBytes())); // named write
                state.scanned = true;
            });

        registerTask(validate, "validate_input",
            [k](const Vec& x) { return yes(x, k.scanned) && !yes(x, k.validated); },
            change({{k.validated, 1}}),
            [&] { job.validate(); state.validated = true; });

        registerTask(prepare, "create_staging",
            [k](const Vec& x) { return yes(x, k.validated) && !yes(x, k.prepared); },
            change({{k.prepared, 1}}),
            [&] { job.prepare(); state.prepared = true; });

        registerTask(fastCopy, "fast_copy",
            [k](const Vec& x) { return yes(x, k.prepared) && !yes(x, k.copied); },
            change({{k.copied, 1}}),
            [&] { job.copyFast(); state.copied = true; });

        registerTask(selectSlow, "select_safe_fallback",
            [k](const Vec& x) {
                return yes(x, k.prepared) && !yes(x, k.copied) && !yes(x, k.safeReady);
            },
            change({{k.safeReady, 1}}),
            [&] { job.selectSlowCopy(); state.safeReady = true; });

        registerTask(slowCopy, "chunked_copy",
            [k](const Vec& x) {
                return yes(x, k.prepared) && yes(x, k.safeReady) && !yes(x, k.copied);
            },
            change({{k.copied, 1}, {k.safeReady, 0}}),
            [&] {
                job.copySlow();
                state.copied = true;
                state.safeReady = false; // return planning vector to common goal
            });

        registerTask(verify, "verify_and_manifest",
            [k](const Vec& x) { return yes(x, k.copied) && !yes(x, k.verified); },
            change({{k.verified, 1}}),
            [&] { job.verifyAndManifest(); state.verified = true; });

        registerTask(publish, "publish_release",
            [k](const Vec& x) { return yes(x, k.verified) && !yes(x, k.published); },
            change({{k.published, 1}}),
            [&] {
                job.publish();
                state.releasePath = job.release.string();
                state.published = true;
            });

        // The space keeps non-owning pointers to these stable Action members.
        reg(scan, validate, prepare, fastCopy, selectSlow, slowCopy, verify, publish);
    }
};

void printPlan(BackupActions& tasks, const Chain& plan, int iteration) {
    std::cout << "\nPLAN " << iteration << ": " << plan.acts.size() << " steps\n";
    int order = 1;
    for (const auto id : plan.acts) {
        Act* task = tasks.get(id);
        if (!task) throw std::logic_error("unknown planned action");
        std::cout << "  " << order++ << ". " << task->name << '\n';
    }
}

void runPipeline(const fs::path& source, const fs::path& output, bool fault) {
    BackupPolicy parameters;
    const Rules rules = rulesOf(parameters); // fixed snapshot for deterministic planning
    std::cout << "ParameterSpace: " << parameters.size() << " policy fields\n";
    for (const auto& e : parameters.layout())
        std::cout << "  " << e.name << " at Vec[" << e.offset << "]\n";

    BackupState state;
    const Indices k = indicesOf(state);
    SnapshotJob job(source, output, rules, fault);
    BackupActions tasks(k, job, state);

    // Both copy paths converge to the SAME goal (safeReady must be reset to 0).
    Vec goal = state.vec();
    goal[k.scanned] = goal[k.validated] = goal[k.prepared] = 1;
    goal[k.copied] = goal[k.verified] = goal[k.published] = 1;

    // Planning READS the action space + the CURRENT StateSpace vector and never
    // advances any state. Execution then runs one action at a time.
    Chain plan = bfs(tasks, state.vec(), goal, {}, 10, 0.0, 1000);
    for (int round = 1; round <= 3; ++round) {
        if (!plan.ok) throw std::runtime_error("cannot plan: " + plan.why);
        printPlan(tasks, plan, round);
        bool retry = false;

        for (std::uint64_t id : plan.acts) {
            Act* action = tasks.get(id);
            if (!action) throw std::logic_error("planner returned an unknown action");

            // Always work from the LATEST measured state (single source of truth).
            const Vec before = state.vec();
            const Vec predicted = action->next(before);   // candidate successor (pure)
            std::cout << "RUN: " << action->name << '\n';
            if (!tasks.run(id, before))
                throw std::runtime_error("action precondition/execution rejected");

            // StateSpace records facts after real I/O; the predicted vector is
            // only a candidate. Compare, then replan from the measured state.
            const Vec actual = state.vec();
            if (sqDist(predicted, actual) > 0.0) {
                if (id != tasks.fastCopy.hash)
                    throw std::runtime_error("fatal task failure: " + job.lastError);
                std::cout << "  prediction != filesystem result; banning fast_copy\n";
                tasks.ban(id);
                plan = bfs(tasks, actual, goal, {}, 10, 0.0, 1000); // replan from the new state
                retry = true;
                break;
            }
        }
        if (!retry) {
            if (!state.published.get()) throw std::runtime_error("pipeline ended without publishing");
            if (!fs::is_regular_file(job.release / "manifest.tsv"))
                throw std::runtime_error("published snapshot manifest is missing");
            std::cout << "\nSUCCESS: " << state.releasePath.get() << '\n'
                      << "files=" << state.fileCount.get()
                      << " bytes=" << state.get<std::int64_t>("totalBytes") << '\n'
                      << "fast_copy banned=" << std::boolalpha
                      << tasks.banned(tasks.fastCopy.hash) << '\n';
            return;
        }
    }
    throw std::runtime_error("too many replans");
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 1) {
            const fs::path workspace = uniquePath(fs::temp_directory_path(), "aurox_v06_demo_");
            const fs::path source = workspace / "input";
            const fs::path output = workspace / "snapshots";
            createDemoSource(source);
            std::cout << "DEMO workspace: " << workspace << '\n';
            std::cout << "A one-time fast-copy failure will be injected.\n";
            runPipeline(source, output, true);
            std::cout << "Inspect published data under: " << output / "releases" << '\n';
            return 0;
        }
        if (argc != 3 && argc != 4)
            throw std::invalid_argument("usage: aurox_snapshot SOURCE_DIR OUTPUT_DIR [--fail-fast-once]");
        if (argc == 4 && Str{argv[3]} != "--fail-fast-once")
            throw std::invalid_argument("unknown option: " + Str{argv[3]});
        runPipeline(fs::path{argv[1]}, fs::path{argv[2]}, argc == 4);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
}
