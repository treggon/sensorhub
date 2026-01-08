
(function () {
    // Lightweight index built from /api/summary
    var sensorsIndex = {};

    // -------- Helpers --------
    function statusClass(s) {
        if (s === "ok") return "ok";
        if (s === "warning") return "warning";
        if (s === "error") return "error";
        return "unknown";
    }
    function setDot(id, status) {
        var el = document.getElementById(id);
        if (el) el.className = "status-dot " + statusClass(status);
    }
    function showError(msg) {
        var ts = document.getElementById("timestamp");
        if (!ts) return;
        ts.textContent = "Error: " + String(msg || "unknown");
        ts.style.color = "#f0ad4e";
    }
    function clearError() {
        var ts = document.getElementById("timestamp");
        if (!ts) return;
        ts.style.color = "";
    }

    // -------- Table render --------
    function renderSensors(sensors) {
        var tbody = document.querySelector("#sensors-table tbody");
        if (!tbody) return;
        while (tbody.firstChild) tbody.removeChild(tbody.firstChild);

        for (var i = 0; i < sensors.length; i++) {
            var s = sensors[i];

            var tr = document.createElement("tr");

            // Status cell
            var tdStatus = document.createElement("td");
            var dot = document.createElement("span");
            dot.className = "status-dot " + statusClass(s.health);
            tdStatus.appendChild(dot);
            tr.appendChild(tdStatus);

            // Name
            var tdName = document.createElement("td");
            tdName.textContent = s.name || s.id;
            tr.appendChild(tdName);

            // Type
            var tdType = document.createElement("td");
            tdType.textContent = s.type || "-";
            tr.appendChild(tdType);

            // Actions
            var tdActions = document.createElement("td");

            var btnDetails = document.createElement("button");
            btnDetails.className = "btn";
            btnDetails.textContent = "Details";
            btnDetails.dataset.sensorId = s.id;
            btnDetails.addEventListener("click", function (ev) {
                var id = ev.currentTarget.dataset.sensorId;
                openSensor(id);
            });
            tdActions.appendChild(btnDetails);

            // Optional "Live" button for cameras
            var isCamera = (s.type === "camera" || s.kind === "camera");
            if (isCamera) {
                var btnLive = document.createElement("button");
                btnLive.className = "btn";
                btnLive.textContent = "Live";
                btnLive.dataset.sensorId = s.id;
                btnLive.addEventListener("click", function (ev) {
                    var camId = ev.currentTarget.dataset.sensorId;
                    openCameraLive(camId);
                });
                tdActions.appendChild(btnLive);
            }

            var aHist = document.createElement("a");
            aHist.className = "btn";
            aHist.textContent = "History";
            aHist.href = (s.urls && s.urls.history) ? s.urls.history : ("/sensors/" + encodeURIComponent(s.id) + "/history");
            aHist.target = "_blank";
            tdActions.appendChild(aHist);

            var aLatest = document.createElement("a");
            aLatest.className = "btn";
            aLatest.textContent = "Latest";
            aLatest.href = (s.urls && s.urls.latest) ? s.urls.latest : ("/sensors/" + encodeURIComponent(s.id) + "/latest");
            aLatest.target = "_blank";
            tdActions.appendChild(aLatest);

            tr.appendChild(tdActions);
            tbody.appendChild(tr);
        }
    }

    // -------- Summary fetch --------
    async function loadSummary() {
        var url = new URL("/api/summary", window.location.origin).toString();
        try {
            var r = await fetch(url, { cache: "no-store" });
            if (!r.ok) { showError("HTTP " + r.status + " " + r.statusText); return; }
            var data = await r.json();

            clearError();

            // System lights + timestamp
            setDot("dot-health", (data.system && data.system.health) || "unknown");
            setDot("dot-ready", (data.system && data.system.ready) || "unknown");
            setDot("dot-video", (data.system && data.system.video) || "unknown");

            var tsEl = document.getElementById("timestamp");
            var tsVal = (data.system && data.system.timestamp) ? (data.system.timestamp * 1000) : Date.now();
            if (tsEl) tsEl.textContent = "Last updated: " + new Date(tsVal).toLocaleString();

            // Index + render
            sensorsIndex = {};
            var sensors = data.sensors || [];
            for (var i = 0; i < sensors.length; i++) {
                sensorsIndex[sensors[i].id] = sensors[i];
            }
            renderSensors(sensors);
        } catch (e) {
            console.error("Summary load failed:", e);
            showError(e && e.message ? e.message : "fetch failed");
        }
    }

    // -------- Sensor modal (with camera support) --------

    async function openSensor(id) {
        showModal("Sensor: " + id, "<div class=\"muted\">Loading...</div>");

        var s = sensorsIndex[id] || {};
        var urls = s.urls || {
            health: "/sensors/" + encodeURIComponent(id) + "/health",
            latest: "/sensors/" + encodeURIComponent(id) + "/latest",
            snapshot_png: "/sensors/" + encodeURIComponent(id) + "/snapshot.png"
        };
        var isCamera = (s.type === "camera" || s.kind === "camera");
        var chunks = [];

        // --- Health ---
        try {
            var hURL = new URL(urls.health, window.location.origin).toString();
            var hResp = await fetch(hURL, { cache: "no-store" });
            var section = "<h4>Health</h4>";
            if (hResp.ok) {
                var hJson = await hResp.json();
                section += "<pre>" + JSON.stringify(hJson, null, 2) + "</pre>";
            } else {
                section += "<div class=\"muted\">HTTP " + hResp.status + " " + hResp.statusText + "</div>";
            }
            chunks.push(section);
        } catch (e) {
            console.error("Health fetch failed:", e);
            chunks.push("<h4>Health</h4><div class=\"muted\">Fetch failed: " + String(e && e.message || e) + "</div>");
        }

        // --- Latest (metadata only) ---
        try {
            var latestUrl = urls.latest;
            if (latestUrl.indexOf("?") === -1) latestUrl += "?";
            if (!/include_meta=/.test(latestUrl)) latestUrl += (latestUrl.endsWith("?") ? "" : "&") + "include_meta=true";
            if (!/include_points=/.test(latestUrl)) latestUrl += "&include_points=false";

            var lURL = new URL(latestUrl, window.location.origin).toString();
            var lResp = await fetch(lURL, { cache: "no-store" });
            var sectionL = "<h4>Latest</h4>";
            if (lResp.ok) {
                var lJson = await lResp.json();
                sectionL += "<pre>" + JSON.stringify(lJson, null, 2) + "</pre>";
            } else if (lResp.status === 404) {
                sectionL += "<div class=\"muted\">No sample yet (404)</div>";
            } else if (lResp.status === 204) {
                sectionL += "<div class=\"muted\">No content (204)</div>";
            } else {
                sectionL += "<div class=\"muted\">HTTP " + lResp.status + " " + lResp.statusText + "</div>";
            }
            chunks.push(sectionL);
        } catch (e) {
            console.error("Latest fetch failed:", e);
            chunks.push("<h4>Latest</h4><div class=\"muted\">Fetch failed: " + String(e && e.message || e) + "</div>");
        }

        // --- Snapshot ---
        if (isCamera) {
            // Cameras: use /video/{camera_id}/snapshot.jpg (GET)
            var camSnap = new URL("/video/" + encodeURIComponent(id) + "/snapshot.jpg", window.location.origin).toString();
            chunks.push(
                "<h4>Camera Snapshot</h4>" +
                "<div><img class=\"snapshot\" src=\"" + camSnap + "?t=" + Date.now() + "\" alt=\"Snapshot\"></div>" +
                "<div class=\"row\" style=\"margin-top:.5rem;\">" +
                "<button class=\"btn\" id=\"btn-live-" + encodeURIComponent(id) + "\">Open Live MJPEG</button>" +
                "</div>"
            );
            // Wire the Live button after rendering
            setTimeout(function () {
                var btnLive = document.getElementById("btn-live-" + id);
                if (btnLive) {
                    btnLive.addEventListener("click", function () {
                        openCameraLive(id, true);
                    });
                }
            }, 0);
        } else {
            // Non-cameras (e.g., LiDAR, GPS): try /sensors/{id}/snapshot.png
            var sURL = new URL(urls.snapshot_png || ("/sensors/" + encodeURIComponent(id) + "/snapshot.png"), window.location.origin).toString();
            // Just render; if the file doesn't exist, the <img> will show broken icon.
            chunks.push("<h4>Snapshot</h4><div><img class=\"snapshot\" src=\"" + sURL + "?t=" + Date.now() + "\" alt=\"Snapshot\"></div>");
        }

        // --- Action links ---
        chunks.push(
            "<div class=\"row\" style=\"margin-top:.5rem;\">" +
            "<a class=\"btn\" href=\"/sensors/" + encodeURIComponent(id) + "/latest_raw\" target=\"_blank\">Latest Raw</a> " +
            "<a class=\"btn\" href=\"/sensors/" + encodeURIComponent(id) + "/history\" target=\"_blank\">History</a>" +
            "</div>"
        );

        // Render modal
        showModal("Sensor: " + id, chunks.join("\n"));
    }


    // -------- Open camera MJPEG (inline in modal or as a fresh modal) --------
    function openCameraLive(id, inline) {
        var mjpegUrl = new URL("/video/" + encodeURIComponent(id) + "/mjpeg", window.location.origin).toString();
        var snapUrl = new URL("/video/" + encodeURIComponent(id) + "/snapshot.jpg", window.location.origin).toString();

        var block = "<h4>Live MJPEG</h4>" +
            "<div><img id=\"mjpeg-" + encodeURIComponent(id) + "\" style=\"width:100%;max-height:360px;border:1px solid #333;border-radius:6px\" " +
            "src=\"" + mjpegUrl + "\" alt=\"Live MJPEG\"></div>" +
            "<div class=\"muted\" style=\"margin-top:.25rem\">If MJPEG doesn’t play, a snapshot will appear.</div>";

        if (inline) {
            // Append to current modal body
            var body = document.getElementById("modal-body");
            if (body) {
                body.insertAdjacentHTML("beforeend", block);
                // Fallback to snapshot if MJPEG load fails
                setTimeout(function () {
                    var img = document.getElementById("mjpeg-" + id);
                    if (img) {
                        img.addEventListener("error", function () {
                            img.src = snapUrl + "?t=" + Date.now();
                        });
                    }
                }, 0);
            }
        } else {
            // Open a fresh modal with live view
            showModal("Camera: " + id, block);
            setTimeout(function () {
                var img = document.getElementById("mjpeg-" + id);
                if (img) {
                    img.addEventListener("error", function () {
                        img.src = snapUrl + "?t=" + Date.now();
                    });
                }
            }, 0);
        }
    }

    // -------- Modal --------
    function showModal(title, html) {
        var mTitle = document.getElementById("modal-title");
        var mBody = document.getElementById("modal-body");
        var back = document.getElementById("backdrop");
        var modal = document.getElementById("modal");
        if (mTitle) mTitle.textContent = title;
        if (mBody) mBody.innerHTML = html;
        if (back) back.classList.add("show");
        if (modal) modal.classList.add("show");
    }
    window.closeModal = function () {
        var back = document.getElementById("backdrop");
        var modal = document.getElementById("modal");
        if (back) back.classList.remove("show");
        if (modal) modal.classList.remove("show");
    };

    // -------- Kickoff --------
    window.addEventListener("load", function () {
        loadSummary();
        setInterval(loadSummary, 2000);
    });
})();
