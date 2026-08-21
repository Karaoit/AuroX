(() => {
    "use strict";

    const $ = selector => document.querySelector(selector);
    const $$ = selector => Array.from(document.querySelectorAll(selector));

    const bgCanvas = $("#bgCanvas");
    const bgCtx = bgCanvas.getContext("2d");
    const mouseCanvas = $("#mouseCanvas");
    const mouseCtx = mouseCanvas.getContext("2d");

    const connectionDot = $("#connectionDot");
    const connectionText = $("#connectionText");
    const eventLog = $("#eventLog");
    const pointerReadout = $("#pointerReadout");

    const metricEngine = $("#metricEngine");
    const metricTaskSequence = $("#metricTaskSequence");
    const metricLayerCount = $("#metricLayerCount");
    const headerEngineState = $("#headerEngineState");
    const headerTaskCount = $("#headerTaskCount");
    const engineMiniDot = $("#engineMiniDot");
    const engineMiniText = $("#engineMiniText");
    const taskViewEngine = $("#taskViewEngine");
    const taskViewLatestId = $("#taskViewLatestId");
    const taskViewBackend = $("#taskViewBackend");

    let mouseX = innerWidth / 2;
    let mouseY = innerHeight / 2;
    let circleX = mouseX;
    let circleY = mouseY;
    let vx = 0;
    let vy = 0;
    let isMouseInside = true;
    let isTouchInput = false;
    let ringPulse = 0;
    let requestSequence = 0;
    let logRecords = [];
    let activeLogFilter = "all";
    let autoRefreshTimer = null;

    const settings = {
        pointer: true,
        background: true,
        compact: false,
        autoRefresh: true,
        refreshInterval: 8
    };

    const REFERENCE_FPS = 60;

    // 更高的最大跟随速度
    const FOLLOW_MAX_SPEED = 150 * REFERENCE_FPS;

    // 更高加速度，使圆圈更快贴近目标
    const FOLLOW_ACCEL = 5 * REFERENCE_FPS * REFERENCE_FPS;

    // 距离越远速度越快
    const FOLLOW_SPEED_FACTOR = 1.45 * REFERENCE_FPS;

    // 鼠标方向突然反转时，降低旧方向惯性
    // 数值越小，转向时摆动越弱
    const FOLLOW_TURN_DAMPING = 0.38;

    // 转向阻尼主要作用范围
    const FOLLOW_TURN_DAMPING_DISTANCE = 18;

    const SNAP_THRESHOLD = 0.8;
    const MAX_FRAME_DELTA = 1 / 15;
    const DOT_RADIUS = 4.5;
    const RING_RADIUS = 19;

    class CommandBus {
        constructor(endpoint) {
            this.endpoint = endpoint;
            this.handlers = new Map();
        }

        on(code, handler) {
            if (!this.handlers.has(code)) {
                this.handlers.set(code, []);
            }

            this.handlers.get(code).push(handler);
        }

        emitLocal(code, response, request) {
            for (const handler of this.handlers.get(code) || []) {
                try {
                    handler(response, request);
                } catch (error) {
                    console.error("Command handler error:", error);
                }
            }
        }

        async send({
            code,
            source = "ui",
            payload = {},
            x = Math.round(mouseX),
            y = Math.round(mouseY),
            layer = 0
        }) {
            const request = {
                id: ++requestSequence,
                code,
                source,
                payload,
                x,
                y,
                layer,
                clientTime: new Date().toISOString()
            };

            appendLog("send", request, true);

            try {
                const response = await fetch(this.endpoint, {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json"
                    },
                    body: JSON.stringify(request)
                });

                let body;

                try {
                    body = await response.json();
                } catch {
                    body = {
                        ok: false,
                        message: `HTTP ${response.status}: response is not JSON`
                    };
                }

                appendLog(
                    "recv",
                    body,
                    response.ok && body.ok !== false
                );

                if (!response.ok || body.ok === false) {
                    throw new Error(
                        body.message || `HTTP ${response.status}`
                    );
                }

                this.emitLocal(
                    code,
                    body,
                    request
                );

                setConnection(true);

                return body;
            } catch (error) {
                setConnection(false);

                appendLog(
                    "error",
                    {
                        code,
                        message: error.message
                    },
                    false
                );

                throw error;
            }
        }
    }

    const commandBus = new CommandBus("/api/command");

    function escapeHtml(value) {
        return String(value).replace(
            /[&<>"']/g,
            char => ({
                "&": "&amp;",
                "<": "&lt;",
                ">": "&gt;",
                '"': "&quot;",
                "'": "&#039;"
            }[char])
        );
    }

    function parsePayload(element) {
        const raw = element.dataset.payload;

        if (!raw) {
            return {};
        }

        try {
            return JSON.parse(raw);
        } catch (error) {
            appendLog(
                "error",
                {
                    message:
                        `Invalid data-payload on ${
                            element.dataset.source ||
                            element.tagName
                        }`,
                    raw
                },
                false
            );

            return {};
        }
    }

    function getLayer(element) {
        const value = Number(
            element.dataset.layer
        );

        return Number.isFinite(value)
            ? value
            : 0;
    }

    function collectCommandsAtPoint(x, y) {
        const candidates = [];
        const seen = new Set();

        for (
            const rawElement of
            document.elementsFromPoint(x, y)
        ) {
            let element = rawElement;

            while (
                element &&
                element !== document.documentElement
            ) {
                if (
                    element.dataset &&
                    element.dataset.command
                ) {
                    if (!seen.has(element)) {
                        seen.add(element);
                        candidates.push(element);
                    }

                    break;
                }

                element = element.parentElement;
            }
        }

        candidates.sort(
            (a, b) =>
                getLayer(b) -
                getLayer(a)
        );

        return candidates;
    }

    async function dispatchPointClick(event) {
        if (event.button !== 0) {
            return;
        }

        /*
         * 触屏 pointerdown 时已经：
         * 1. 直接瞬移圆圈
         * 2. 播放点击动画
         *
         * 部分浏览器随后还会额外触发 click。
         * 这里跳过一次动画，避免重复播放。
         */
        if (isTouchInput) {
            isTouchInput = false;
        } else {
            startRingAnimation(
                event.clientX,
                event.clientY
            );
        }

        const elements =
            collectCommandsAtPoint(
                event.clientX,
                event.clientY
            );

        metricLayerCount.textContent =
            String(elements.length);

        if (!elements.length) {
            return;
        }

        for (const element of elements) {
            await commandBus.send({
                code:
                    element.dataset.command,

                source:
                    element.dataset.source ||
                    element.id ||
                    element.className ||
                    element.tagName,

                payload:
                    parsePayload(element),

                x:
                    Math.round(
                        event.clientX
                    ),

                y:
                    Math.round(
                        event.clientY
                    ),

                layer:
                    getLayer(element)
            }).catch(() => {});
        }
    }

    function setConnection(online) {
        connectionDot.classList.toggle(
            "online",
            online
        );

        connectionText.textContent =
            online
                ? "Backend Online"
                : "Backend Offline";

        if (taskViewBackend) {
            taskViewBackend.textContent =
                online
                    ? "Online"
                    : "Offline";
        }
    }

    function setEngineState(running) {
        const state =
            running
                ? "RUNNING"
                : "STOPPED";

        metricEngine.textContent =
            state;

        headerEngineState.textContent =
            state;

        taskViewEngine.textContent =
            state;

        engineMiniText.textContent =
            running
                ? "Running"
                : "Stopped";

        engineMiniDot.classList.toggle(
            "running",
            running
        );
    }

    function setTaskSequence(value) {
        const text =
            String(value ?? 0);

        metricTaskSequence.textContent =
            text;

        headerTaskCount.textContent =
            text;

        taskViewLatestId.textContent =
            text;
    }

    function appendLog(type, data, ok) {
        const record = {
            time: new Date(),
            type,
            ok,

            code:
                data && data.code
                    ? String(data.code)
                    : "",

            message:
                data && data.message
                    ? String(data.message)
                    : JSON.stringify(data)
        };

        logRecords.unshift(record);

        if (logRecords.length > 200) {
            logRecords.pop();
        }

        renderLogs();
    }

    function renderLogs() {
        if (!eventLog) {
            return;
        }

        const visible =
            logRecords.filter(
                record =>
                    activeLogFilter === "all" ||
                    record.type === activeLogFilter
            );

        eventLog.innerHTML =
            visible.map(record => `
                <div
                    class="log-entry ${
                        record.ok
                            ? record.type
                            : "error"
                    }"
                    data-type="${
                        escapeHtml(record.type)
                    }"
                >
                    <div class="log-meta">
                        ${
                            escapeHtml(
                                record.time.toLocaleTimeString()
                            )
                        }
                    </div>

                    <div class="log-message">
                        ${
                            record.code
                                ? `<span class="log-code">${
                                    escapeHtml(record.code)
                                }</span> `
                                : ""
                        }

                        ${
                            escapeHtml(
                                record.message
                            )
                        }
                    </div>

                    <span class="log-kind">
                        ${
                            escapeHtml(
                                record.type.toUpperCase()
                            )
                        }
                    </span>
                </div>
            `).join("");

        $("#logCount").textContent =
            `${logRecords.length} events`;
    }

    function setActiveView(view) {
        $$(".nav-item").forEach(
            button => {
                button.classList.toggle(
                    "active",
                    button.dataset.view === view
                );
            }
        );

        $$(".view-page").forEach(
            page => {
                page.classList.toggle(
                    "active",
                    page.dataset.page === view
                );
            }
        );

        $("#footerRoute").textContent =
            `VIEW / ${view.toUpperCase()}`;
    }

    commandBus.on(
        "SYS.STATUS",
        response => {
            setEngineState(
                Boolean(
                    response.data?.engineRunning
                )
            );

            setTaskSequence(
                response.data?.taskSequence ?? 0
            );
        }
    );

    commandBus.on(
        "ENGINE.START",
        response =>
            setEngineState(
                Boolean(
                    response.data?.engineRunning
                )
            )
    );

    commandBus.on(
        "ENGINE.STOP",
        response =>
            setEngineState(
                Boolean(
                    response.data?.engineRunning
                )
            )
    );

    commandBus.on(
        "TASK.CREATE",
        response => {
            if (
                response.data?.taskId !==
                undefined
            ) {
                setTaskSequence(
                    response.data.taskId
                );
            }
        }
    );

    for (
        const view of [
            "OVERVIEW",
            "TASKS",
            "LOGS",
            "SETTINGS"
        ]
    ) {
        commandBus.on(
            `VIEW.${view}`,
            () =>
                setActiveView(
                    view.toLowerCase()
                )
        );
    }

    $$(".nav-item").forEach(
        button => {
            button.addEventListener(
                "click",
                () =>
                    setActiveView(
                        button.dataset.view
                    )
            );
        }
    );

    $("#sendManualBtn").addEventListener(
        "click",
        async event => {
            event.stopPropagation();

            const code =
                $("#commandCodeInput")
                    .value
                    .trim();

            const payloadText =
                $("#payloadInput")
                    .value
                    .trim();

            if (!code) {
                appendLog(
                    "error",
                    {
                        message:
                            "Command code is empty"
                    },
                    false
                );

                return;
            }

            let payload = {};

            try {
                payload =
                    payloadText
                        ? JSON.parse(
                            payloadText
                        )
                        : {};
            } catch (error) {
                appendLog(
                    "error",
                    {
                        message:
                            `Payload JSON error: ${
                                error.message
                            }`
                    },
                    false
                );

                return;
            }

            await commandBus.send({
                code,
                source:
                    "manual.composer",
                payload,
                layer: 170
            }).catch(() => {});
        }
    );

    $("#formatPayloadBtn").addEventListener(
        "click",
        event => {
            event.stopPropagation();

            try {
                const parsed =
                    JSON.parse(
                        $("#payloadInput").value ||
                        "{}"
                    );

                $("#payloadInput").value =
                    JSON.stringify(
                        parsed,
                        null,
                        2
                    );
            } catch (error) {
                appendLog(
                    "error",
                    {
                        message:
                            `Payload JSON error: ${
                                error.message
                            }`
                    },
                    false
                );
            }
        }
    );

    $("#clearLogBtn").addEventListener(
        "click",
        event => {
            event.stopPropagation();

            logRecords = [];
            renderLogs();
        }
    );

    $("#exportLogBtn").addEventListener(
        "click",
        event => {
            event.stopPropagation();

            const content =
                logRecords
                    .slice()
                    .reverse()
                    .map(
                        record =>
                            `[${record.time.toISOString()}] ${
                                record.type.toUpperCase()
                            } ${
                                record.code
                            } ${
                                record.message
                            }`
                    )
                    .join("\n");

            const blob =
                new Blob(
                    [content],
                    {
                        type:
                            "text/plain;charset=utf-8"
                    }
                );

            const url =
                URL.createObjectURL(blob);

            const anchor =
                document.createElement("a");

            anchor.href = url;

            anchor.download =
                "auroX-command-log.txt";

            anchor.click();

            URL.revokeObjectURL(url);
        }
    );

    $$("[data-log-filter]").forEach(
        button => {
            button.addEventListener(
                "click",
                event => {
                    event.stopPropagation();

                    activeLogFilter =
                        button.dataset.logFilter;

                    $$(
                        "[data-log-filter]"
                    ).forEach(
                        item =>
                            item.classList.toggle(
                                "active",
                                item === button
                            )
                    );

                    renderLogs();
                }
            );
        }
    );

    let activeOperatorFilter =
        "all";

    function applyOperatorFilter() {
        const keyword =
            $("#operatorSearch")
                .value
                .trim()
                .toLowerCase();

        $$(".operator-item").forEach(
            item => {
                const matchCategory =
                    activeOperatorFilter ===
                        "all" ||
                    item.dataset.category ===
                        activeOperatorFilter;

                const searchable =
                    `${
                        item.dataset.name
                    } ${
                        item.dataset.category
                    } ${
                        item.dataset.description
                    }`
                    .toLowerCase();

                item.hidden =
                    !(
                        matchCategory &&
                        searchable.includes(
                            keyword
                        )
                    );
            }
        );
    }

    $$(
        "#operatorFilters .filter-chip"
    ).forEach(
        button => {
            button.addEventListener(
                "click",
                event => {
                    event.stopPropagation();

                    activeOperatorFilter =
                        button.dataset.filter;

                    $$(
                        "#operatorFilters .filter-chip"
                    ).forEach(
                        item =>
                            item.classList.toggle(
                                "active",
                                item === button
                            )
                    );

                    applyOperatorFilter();
                }
            );
        }
    );

    $("#operatorSearch")
        .addEventListener(
            "input",
            applyOperatorFilter
        );

    $("#clearOperatorSearch")
        .addEventListener(
            "click",
            event => {
                event.stopPropagation();

                $("#operatorSearch").value =
                    "";

                activeOperatorFilter =
                    "all";

                $$(
                    "#operatorFilters .filter-chip"
                ).forEach(
                    item =>
                        item.classList.toggle(
                            "active",
                            item.dataset.filter ===
                                "all"
                        )
                );

                applyOperatorFilter();
            }
        );

    function selectOperator(item) {
        $$(".operator-item").forEach(
            node =>
                node.classList.toggle(
                    "active",
                    node === item
                )
        );

        $("#inspectorName").textContent =
            item.dataset.name;

        $("#inspectorCategory").textContent =
            item.dataset.category.toUpperCase();

        $("#inspectorSignature").textContent =
            item.dataset.signature;

        $("#inspectorDescription").textContent =
            item.dataset.description;

        try {
            $("#inspectorParams").value =
                JSON.stringify(
                    JSON.parse(
                        item.dataset.params ||
                        "{}"
                    ),
                    null,
                    2
                );
        } catch {
            $("#inspectorParams").value =
                "{}";
        }
    }

    $$(".operator-item").forEach(
        item => {
            item.addEventListener(
                "click",
                event => {
                    event.stopPropagation();

                    selectOperator(item);
                }
            );
        }
    );

    $("#loadOperatorToComposer")
        .addEventListener(
            "click",
            event => {
                event.stopPropagation();

                let config = {};

                try {
                    config =
                        JSON.parse(
                            $("#inspectorParams")
                                .value ||
                            "{}"
                        );
                } catch (error) {
                    appendLog(
                        "error",
                        {
                            message:
                                `Operator parameters JSON error: ${
                                    error.message
                                }`
                        },
                        false
                    );

                    return;
                }

                $("#commandCodeInput").value =
                    "TASK.CREATE";

                $("#payloadInput").value =
                    JSON.stringify(
                        {
                            type:
                                "operator-pipeline",

                            parameters: {
                                operator:
                                    $("#inspectorName")
                                        .textContent,

                                config
                            }
                        },
                        null,
                        2
                    );

                setActiveView("tasks");
            }
        );

    $("#copyOperatorConfig")
        .addEventListener(
            "click",
            async event => {
                event.stopPropagation();

                const text =
                    $("#inspectorParams")
                        .value;

                try {
                    await navigator.clipboard
                        .writeText(text);

                    appendLog(
                        "ui",
                        {
                            message:
                                "Operator config copied to clipboard"
                        },
                        true
                    );
                } catch {
                    appendLog(
                        "error",
                        {
                            message:
                                "Clipboard API unavailable"
                        },
                        false
                    );
                }
            }
        );

    function loadSettings() {
        try {
            const stored =
                JSON.parse(
                    localStorage.getItem(
                        "aurox.ui.settings"
                    ) ||
                    "{}"
                );

            Object.assign(
                settings,
                stored
            );
        } catch {}

        $("#pointerToggle").checked =
            settings.pointer;

        $("#backgroundToggle").checked =
            settings.background;

        $("#compactToggle").checked =
            settings.compact;

        $("#autoRefreshToggle").checked =
            settings.autoRefresh;

        $("#refreshIntervalInput").value =
            settings.refreshInterval;

        applySettings();
    }

    function applySettings() {
        document.body.classList.toggle(
            "pointer-disabled",
            !settings.pointer
        );

        document.body.classList.toggle(
            "compact",
            settings.compact
        );

        mouseCanvas.style.display =
            settings.pointer
                ? "block"
                : "none";

        setupAutoRefresh();
    }

    function setupAutoRefresh() {
        clearInterval(
            autoRefreshTimer
        );

        autoRefreshTimer = null;

        if (!settings.autoRefresh) {
            return;
        }

        const seconds =
            Math.max(
                2,
                Math.min(
                    60,
                    Number(
                        settings.refreshInterval
                    ) || 8
                )
            );

        autoRefreshTimer =
            setInterval(
                () =>
                    refreshStatus(
                        "app.auto-refresh"
                    ),
                seconds * 1000
            );
    }

    $("#saveSettingsBtn")
        .addEventListener(
            "click",
            event => {
                event.stopPropagation();

                settings.pointer =
                    $("#pointerToggle")
                        .checked;

                settings.background =
                    $("#backgroundToggle")
                        .checked;

                settings.compact =
                    $("#compactToggle")
                        .checked;

                settings.autoRefresh =
                    $("#autoRefreshToggle")
                        .checked;

                settings.refreshInterval =
                    Math.max(
                        2,
                        Math.min(
                            60,
                            Number(
                                $("#refreshIntervalInput")
                                    .value
                            ) || 8
                        )
                    );

                localStorage.setItem(
                    "aurox.ui.settings",
                    JSON.stringify(
                        settings
                    )
                );

                applySettings();

                appendLog(
                    "ui",
                    {
                        message:
                            "UI settings saved"
                    },
                    true
                );
            }
        );

    document.addEventListener(
        "click",
        dispatchPointClick,
        true
    );

    function resizeCanvas() {
        const dpr =
            Math.min(
                window.devicePixelRatio ||
                1,
                2
            );

        const width =
            innerWidth;

        const height =
            innerHeight;

        for (
            const canvas of [
                bgCanvas,
                mouseCanvas
            ]
        ) {
            canvas.width =
                Math.floor(
                    width * dpr
                );

            canvas.height =
                Math.floor(
                    height * dpr
                );

            canvas.style.width =
                `${width}px`;

            canvas.style.height =
                `${height}px`;

            const ctx =
                canvas.getContext("2d");

            ctx.setTransform(
                dpr,
                0,
                0,
                dpr,
                0,
                0
            );
        }
    }

    function drawBackground() {
        const w =
            innerWidth;

        const h =
            innerHeight;

        bgCtx.clearRect(
            0,
            0,
            w,
            h
        );

        if (!settings.background) {
            bgCtx.fillStyle =
                "#120608";

            bgCtx.fillRect(
                0,
                0,
                w,
                h
            );

            return;
        }

        const x =
            Math.max(
                0,
                Math.min(
                    w,
                    mouseX
                )
            );

        const y =
            Math.max(
                0,
                Math.min(
                    h,
                    mouseY
                )
            );

        const radius =
            Math.min(
                900,
                Math.max(
                    440,
                    Math.min(
                        w,
                        h
                    ) * 0.95
                )
            );

        const gradient =
            bgCtx.createRadialGradient(
                x,
                y,
                0,
                x,
                y,
                radius
            );

        gradient.addColorStop(
            0,
            "#852532"
        );

        gradient.addColorStop(
            0.27,
            "#57161f"
        );

        gradient.addColorStop(
            0.62,
            "#2d0b10"
        );

        gradient.addColorStop(
            1,
            "#100305"
        );

        bgCtx.fillStyle =
            gradient;

        bgCtx.fillRect(
            0,
            0,
            w,
            h
        );
    }

    function updateCirclePhysics(dt) {
        if (
            !settings.pointer ||
            !isMouseInside ||
            dt <= 0
        ) {
            return;
        }

        const dx =
            mouseX - circleX;

        const dy =
            mouseY - circleY;

        const distance =
            Math.hypot(
                dx,
                dy
            );

        const snapSpeed =
            0.8 * REFERENCE_FPS;

        /*
         * 已经非常接近鼠标位置时直接吸附。
         * 防止极小距离时持续抖动。
         */
        if (
            distance <
                SNAP_THRESHOLD &&
            Math.abs(vx) <
                snapSpeed &&
            Math.abs(vy) <
                snapSpeed
        ) {
            circleX =
                mouseX;

            circleY =
                mouseY;

            vx = 0;
            vy = 0;

            return;
        }

        if (distance < 0.001) {
            return;
        }

        const dirX =
            dx / distance;

        const dirY =
            dy / distance;

        /*
         * --------------------------------------
         * 转向阻尼
         * --------------------------------------
         *
         * 当圆圈当前运动方向和新的鼠标方向相反时，
         * 说明鼠标发生了明显转向。
         *
         * 此时快速削弱旧速度，
         * 避免圆圈因为惯性冲过目标，
         * 再反方向冲回来，
         * 从而减少左右来回摆动。
         */
        const speed =
            Math.hypot(
                vx,
                vy
            );

        if (speed > 0.001) {
            const velocityDirX =
                vx / speed;

            const velocityDirY =
                vy / speed;

            const alignment =
                velocityDirX * dirX +
                velocityDirY * dirY;

            /*
             * alignment:
             *
             *  1   = 完全同方向
             *  0   = 垂直
             * -1   = 完全反方向
             */
            if (alignment < 0) {
                const distanceFactor =
                    Math.min(
                        1,
                        distance /
                            FOLLOW_TURN_DAMPING_DISTANCE
                    );

                const damping =
                    1 -
                    (
                        1 -
                        FOLLOW_TURN_DAMPING
                    ) *
                    (-alignment) *
                    distanceFactor;

                vx *= damping;
                vy *= damping;
            }
        }

        /*
         * 目标速度：
         * 距离越大，速度越快。
         */
        const targetSpeed =
            Math.min(
                FOLLOW_MAX_SPEED,
                distance *
                    FOLLOW_SPEED_FACTOR
            );

        const desiredVx =
            dirX * targetSpeed;

        const desiredVy =
            dirY * targetSpeed;

        const dvx =
            desiredVx - vx;

        const dvy =
            desiredVy - vy;

        const dvMagnitude =
            Math.hypot(
                dvx,
                dvy
            );

        /*
         * 限制每帧最大速度变化，
         * 保留一定吸附动画，
         * 但响应明显比原版更快。
         */
        const maxDeltaV =
            FOLLOW_ACCEL * dt;

        const scale =
            dvMagnitude >
            maxDeltaV
                ? maxDeltaV /
                    dvMagnitude
                : 1;

        vx +=
            dvx * scale;

        vy +=
            dvy * scale;

        /*
         * 最终速度限制。
         */
        const limitedSpeed =
            Math.hypot(
                vx,
                vy
            );

        if (
            limitedSpeed >
            FOLLOW_MAX_SPEED
        ) {
            vx =
                vx /
                limitedSpeed *
                FOLLOW_MAX_SPEED;

            vy =
                vy /
                limitedSpeed *
                FOLLOW_MAX_SPEED;
        }

        circleX +=
            vx * dt;

        circleY +=
            vy * dt;
    }

    /*
     * 点击动画。
     *
     * snapToPoint=true：
     * 直接把圆圈瞬移到指定位置，
     * 主要用于触屏点击。
     */
    function startRingAnimation(
        x = mouseX,
        y = mouseY,
        snapToPoint = false
    ) {
        if (!settings.pointer) {
            return;
        }

        if (snapToPoint) {
            mouseX = x;
            mouseY = y;

            circleX = x;
            circleY = y;

            vx = 0;
            vy = 0;
        }

        ringPulse = 1;
    }

    function drawPointer() {
        mouseCtx.clearRect(
            0,
            0,
            innerWidth,
            innerHeight
        );

        if (
            !settings.pointer ||
            !isMouseInside
        ) {
            return;
        }

        ringPulse *= 0.84;

        if (ringPulse < 0.01) {
            ringPulse = 0;
        }

        const ringRadius =
            RING_RADIUS *
            (
                1 -
                ringPulse * 0.45
            );

        /*
         * 外圈
         */
        mouseCtx.beginPath();

        mouseCtx.arc(
            circleX,
            circleY,
            ringRadius,
            0,
            Math.PI * 2
        );

        mouseCtx.strokeStyle =
            "rgba(255,255,255,.88)";

        mouseCtx.lineWidth =
            2;

        mouseCtx.stroke();

        /*
         * 鼠标中心点
         */
        mouseCtx.beginPath();

        mouseCtx.arc(
            mouseX,
            mouseY,
            DOT_RADIUS,
            0,
            Math.PI * 2
        );

        mouseCtx.fillStyle =
            "#ffffff";

        mouseCtx.fill();

        /*
         * 中心柔光
         */
        mouseCtx.beginPath();

        mouseCtx.arc(
            mouseX,
            mouseY,
            DOT_RADIUS - 1.6,
            0,
            Math.PI * 2
        );

        mouseCtx.fillStyle =
            "rgba(255,185,198,.72)";

        mouseCtx.fill();
    }

    let lastFrameTime =
        null;

    function animate(timestamp) {
        if (
            lastFrameTime === null
        ) {
            lastFrameTime =
                timestamp;
        }

        let dt =
            Math.max(
                (
                    timestamp -
                    lastFrameTime
                ) /
                1000,
                0
            );

        lastFrameTime =
            timestamp;

        dt =
            Math.min(
                dt,
                MAX_FRAME_DELTA
            );

        updateCirclePhysics(dt);

        drawBackground();

        drawPointer();

        requestAnimationFrame(
            animate
        );
    }

    /*
     * ======================================
     * Touch / Pointer
     * ======================================
     *
     * 触屏位置可能从上一次鼠标坐标
     * 突然跳到新的触点。
     *
     * 所以 touch pointerdown 时：
     *
     * 1. mouse 坐标直接变为触点
     * 2. circle 坐标直接变为触点
     * 3. 清空惯性速度
     * 4. 在触点原地播放点击动画
     *
     * 不产生圆圈跨屏移动动画。
     */
    document.addEventListener(
        "pointerdown",
        event => {
            if (
                event.pointerType !==
                    "touch" ||
                event.button !== 0
            ) {
                return;
            }

            isTouchInput = true;
            isMouseInside = true;

            startRingAnimation(
                event.clientX,
                event.clientY,
                true
            );

            pointerReadout.textContent =
                `x: ${
                    Math.round(mouseX)
                } / y: ${
                    Math.round(mouseY)
                }`;
        },
        true
    );

    /*
     * 手指拖动时也直接跟随触点，
     * 不播放弹性吸附动画。
     *
     * 这是为了避免触屏设备上
     * PointerEvent 坐标间隔较大时，
     * 圆圈出现明显滞后。
     */
    document.addEventListener(
        "pointermove",
        event => {
            if (
                event.pointerType !==
                "touch"
            ) {
                return;
            }

            isTouchInput = true;
            isMouseInside = true;

            mouseX =
                event.clientX;

            mouseY =
                event.clientY;

            circleX =
                mouseX;

            circleY =
                mouseY;

            vx = 0;
            vy = 0;

            pointerReadout.textContent =
                `x: ${
                    Math.round(mouseX)
                } / y: ${
                    Math.round(mouseY)
                }`;
        },
        true
    );

    /*
     * 鼠标保持原本的弹性跟随效果。
     */
    document.addEventListener(
        "mousemove",
        event => {
            isTouchInput = false;
            isMouseInside = true;

            mouseX =
                event.clientX;

            mouseY =
                event.clientY;

            pointerReadout.textContent =
                `x: ${
                    Math.round(mouseX)
                } / y: ${
                    Math.round(mouseY)
                }`;
        }
    );

    document.addEventListener(
        "mouseenter",
        () => {
            isMouseInside = true;
        }
    );

    document.addEventListener(
        "mouseleave",
        () => {
            isMouseInside = false;
        }
    );

    window.addEventListener(
        "resize",
        resizeCanvas
    );

    async function refreshStatus(
        source = "app.init"
    ) {
        await commandBus.send({
            code:
                "SYS.STATUS",

            source,

            payload: {},

            layer: 0
        }).catch(() => {});
    }

    resizeCanvas();

    loadSettings();

    renderLogs();

    setActiveView(
        "overview"
    );

    requestAnimationFrame(
        animate
    );

    refreshStatus();
})();
