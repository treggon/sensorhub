
(function () {
    // Indexes built from /api/summary and /video/cameras
    var sensorsIndex = {};
    var cameraInfo = {};  // id -> { width: number, height: number }

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

    // Apply aspect ratio to an <img> by known dims (for cameras)
    function applyAspectRatio(imgEl, dims) {
        if (!imgEl || !dims || !dims.width || !dims.height) return;
        imgEl.style.aspectRatio = String(dims.width) + " / " + String(dims.height);
        imgEl.style.width = "100%";
        imgEl.style.height = "auto";
        imgEl.style.objectFit = "contain";
        imgEl.style.border = "1px solid #333";
        imgEl.style.borderRadius = "6px";
        imgEl.style.maxHeight = "70vh";
        imgEl.style.display = "block";
    }

    // Detect camera dims by loading the snapshot if /video/cameras lacks them
    function detectCameraDimsViaSnapshot(id, snapshotUrl, cb) {
        try {
            var probe = new Image();
            probe.onload = function () {
                var w = probe.naturalWidth || 0;
                var h = probe.naturalHeight || 0;
                if (w > 0 && h > 0) {
                    cameraInfo[id] = { width: w, height: h };
                    if (typeof cb === "function") cb({ width: w, height: h });
                } else {
                    if (typeof cb === "function") cb(null);
                }
            };
            probe.onerror = function () { if (typeof cb === "function") cb(null); };
            probe.src = snapshotUrl + "?t=" + Date.now();
        } catch (e) {
            if (typeof cb === "function") cb(null);
        }
    }

    function getCameraDims(id, snapshotUrl, cb) {
        var info = cameraInfo[id];
        if (info && info.width && info.height) {
            cb(info);
            return;
        }
        detectCameraDimsViaSnapshot(id, snapshotUrl, cb);
    }

    // Insert snapshot block at the very top of modal and auto‑retry through candidates on error
    function mountSnapshotAtTop(imgId, titleText, urlCandidates, afterInsertCb) {
        var body = document.getElementById("modal-body");
        if (!body || !urlCandidates || !urlCandidates.length) return;

        var html =
            "<h4>" + titleText + "</h4>" +
            "<div><img id=\"" + imgId + "\" class=\"snapshot\" src=\"" + urlCandidates[0] + "?t=" + Date.now() + "\" alt=\"Snapshot\"></div>";
        // Put snapshot at the top of the modal
        body.insertAdjacentHTML("afterbegin", html);

        setTimeout(function () {
            var img = document.getElementById(imgId);
            if (!img) return;

            // Hide until a successful load to avoid broken icon flashes
            img.style.visibility = "hidden";
            img.addEventListener("load", function () { img.style.visibility = "visible"; });

            var idx = 0;
            img.addEventListener("error", function () {
                idx += 1;
                if (idx < urlCandidates.length) {
                    img.src = urlCandidates[idx] + "?t=" + Date.now();
                }
            });

            try { if (typeof afterInsertCb === "function") afterInsertCb(img); } catch (_) { }
        }, 0);
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
            tdType.textContent = s.type || s.kind || "-";
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

            var isCamera = (s.type === "camera" || s.kind === "camera");
            if (isCamera) {
                var btnLive = document.createElement("button");
                btnLive.className = "btn";
                btnLive.textContent = "Live";
                btnLive.dataset.sensorId = s.id;
                btnLive.addEventListener("click", function (ev) {
                    var camId = ev.currentTarget.dataset.sensorId;
                    openCameraLive(camId, true);
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

            // Index sensors
            sensorsIndex = {};
            var sensors = data.sensors || [];
            for (var i = 0; i < sensors.length; i++) {
                sensorsIndex[sensors[i].id] = sensors[i];
            }

            // Fetch /video/cameras to cache width/height (best effort)
            try {
                var camsResp = await fetch(new URL("/video/cameras", window.location.origin).toString(), { cache: "no-store" });
                if (camsResp.ok) {
                    var cams = await camsResp.json();
                    var list = Array.isArray(cams) ? cams : (cams && cams.cameras ? cams.cameras : []);
                    for (var j = 0; j < list.length; j++) {
                        var c = list[j] || {};
                        var cid = c.id || c.camera_id || c.name;
                        var w = c.width || c.w || c.frame_width;
                        var h = c.height || c.h || c.frame_height;
                        if (cid && w && h) {
                            cameraInfo[cid] = { width: Number(w), height: Number(h) };
                        }
                    }
                }
            } catch (e) {
                console.warn("Failed to load /video/cameras:", e);
            }

            renderSensors(sensors);
        } catch (e) {
            console.error("Summary load failed:", e);
            showError(e && e.message ? e.message : "fetch failed");
        }
    }

    // -------- Sensor modal (snapshots first at top; then health/latest/actions) --------
    async function openSensor(id) {
        // Start with an empty modal body
        showModal("Sensor: " + id, "");

        var s = sensorsIndex[id] || {};
        var urls = s.urls || {
            health: "/sensors/" + encodeURIComponent(id) + "/health",
            latest: "/sensors/" + encodeURIComponent(id) + "/latest",
            snapshot_png: "/sensors/" + encodeURIComponent(id) + "/snapshot.png"
        };

        var kindStr = String(s.type || s.kind || "").toLowerCase();
        var isCamera = (s.type === "camera" || s.kind === "camera" || kindStr === "camera");
        var isLivox = (id === "livox") || (kindStr.indexOf("livox") >= 0);
        var isRplidar = (kindStr.indexOf("lidar2d") >= 0) || (id.indexOf("rplidar") >= 0);

        var body = document.getElementById("modal-body");

        // 1) SNAPSHOT (top of modal)
        if (isCamera) {
            var camSnap = new URL("/video/" + encodeURIComponent(id) + "/snapshot.jpg", window.location.origin).toString();
            mountSnapshotAtTop("cam-snap-" + encodeURIComponent(id), "Camera Snapshot", [camSnap], function (img) {
                var dims = cameraInfo[id];
                if (dims && dims.width && dims.height) {
                    applyAspectRatio(img, dims);
                } else {
                    getCameraDims(id, camSnap, function (found) { applyAspectRatio(img, found || null); });
                }
            });

            if (body) {
                body.insertAdjacentHTML("beforeend",
                    "<div class=\"row\" style=\"margin-top:.5rem;\">" +
                    "<button class=\"btn\" id=\"btn-live-" + encodeURIComponent(id) + "\">Open Live MJPEG</button>" +
                    "</div>"
                );
                setTimeout(function () {
                    var btnLive = document.getElementById("btn-live-" + id);
                    if (btnLive) btnLive.addEventListener("click", function () { openCameraLive(id, true); });
                }, 0);
            }
        }
        else if (isLivox) {
            // Try sensors path first, then livox path(s), png/jpg
            var livoxCandidates = [
                new URL("/sensors/livox/snapshot.png", window.location.origin).toString(),
                new URL("/sensors/livox/snapshot.jpg", window.location.origin).toString(),
                new URL("/livox/snapshot.png", window.location.origin).toString(),
                new URL("/livox/snapshot.jpg", window.location.origin).toString()
            ];
            mountSnapshotAtTop("livox-snap", "Livox Snapshot", livoxCandidates, function (img) {
                img.style.width = "100%";
                img.style.height = "auto";
                img.style.objectFit = "contain";
                img.style.border = "1px solid #333";
                img.style.borderRadius = "6px";
                img.style.maxHeight = "70vh";
                img.style.display = "block";
            });

            // Height (Z) summary right under the snapshot
            setTimeout(async function () {
                var img = document.getElementById("livox-snap");
                if (img && img.parentElement) {
                    img.parentElement.insertAdjacentHTML("afterend", "<div id=\"livox-height\" class=\"muted\" style=\"margin-top:.5rem;\">Computing height (Z)…</div>");
                } else if (body) {
                    body.insertAdjacentHTML("beforeend", "<div id=\"livox-height\" class=\"muted\" style=\"margin-top:.5rem;\">Computing height (Z)…</div>");
                }

                var heightEl = document.getElementById("livox-height");
                try {
                    var fURL = new URL("/sensors/livox/frame?max_points=10000&keep=xyz&decimals=2", window.location.origin).toString();
                    var fResp = await fetch(fURL, { cache: "no-store" });
                    if (fResp.ok) {
                        var fJson = await fResp.json();
                        var pts = fJson && fJson.points ? fJson.points : [];
                        var minZ = null, maxZ = null;
                        for (var i = 0; i < pts.length; i++) {
                            var p = pts[i];
                            var z = (Array.isArray(p) && p.length >= 3) ? Number(p[2]) : null;
                            if (z === null || isNaN(z)) continue;
                            if (minZ === null || z < minZ) minZ = z;
                            if (maxZ === null || z > maxZ) maxZ = z;
                        }
                        if (minZ === null || maxZ === null) {
                            if (heightEl) heightEl.textContent = "Height (Z): not available";
                        } else {
                            if (heightEl) heightEl.textContent = "Height (Z): min " + minZ + " m, max " + maxZ + " m";
                        }
                    } else {
                        if (heightEl) heightEl.textContent = "Height (Z): HTTP " + fResp.status + " " + fResp.statusText;
                    }
                } catch (e) {
                    if (heightEl) heightEl.textContent = "Height (Z): compute failed";
                    console.warn("Livox height compute failed:", e);
                }
            }, 0);
        }
        else if (isRplidar) {
            var genericCandidatesR = [
                new URL("/sensors/" + encodeURIComponent(id) + "/snapshot.png", window.location.origin).toString(),
                new URL("/sensors/" + encodeURIComponent(id) + "/snapshot.jpg", window.location.origin).toString()
            ];
            mountSnapshotAtTop("snap-" + encodeURIComponent(id), "RPLIDAR Snapshot", genericCandidatesR, function (img) {
                img.style.width = "100%";
                img.style.height = "auto";
                img.style.objectFit = "contain";
                img.style.border = "1px solid #333";
                img.style.borderRadius = "6px";
                img.style.maxHeight = "70vh";
                img.style.display = "block";
            });
        }
        else {
            var genericCandidates = [
                new URL("/sensors/" + encodeURIComponent(id) + "/snapshot.png", window.location.origin).toString(),
                new URL("/sensors/" + encodeURIComponent(id) + "/snapshot.jpg", window.location.origin).toString()
            ];
            mountSnapshotAtTop("snap-" + encodeURIComponent(id), "Snapshot", genericCandidates, function (img) {
                img.style.width = "100%";
                img.style.height = "auto";
                img.style.objectFit = "contain";
                img.style.border = "1px solid #333";
                img.style.borderRadius = "6px";
                img.style.maxHeight = "70vh";
                img.style.display = "block";
            });
        }

        // 2) HEALTH
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
            if (body) body.insertAdjacentHTML("beforeend", section);
        } catch (e) {
            console.error("Health fetch failed:", e);
            if (body) body.insertAdjacentHTML("beforeend", "<h4>Health</h4><div class=\"muted\">Fetch failed: " + String(e && e.message || e) + "</div>");
        }

        // 3) LATEST (metadata only)
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
            if (body) body.insertAdjacentHTML("beforeend", sectionL);
        } catch (e) {
            console.error("Latest fetch failed:", e);
            if (body) body.insertAdjacentHTML("beforeend", "<h4>Latest</h4><div class=\"muted\">Fetch failed: " + String(e && e.message || e) + "</div>");
        }

        // 4) ACTION LINKS
        if (body) {
            body.insertAdjacentHTML("beforeend",
                "<div class=\"row\" style=\"margin-top:.5rem;\">" +
                "<a class=\"btn\" href=\"/sensors/" + encodeURIComponent(id) + "/latest_raw\" target=\"_blank\">Latest Raw</a> " +
                "<a class=\"btn\" href=\"/sensors/" + encodeURIComponent(id) + "/history\" target=\"_blank\">History</a>" +
                "</div>"
            );
        }
    }

    // -------- Open camera MJPEG (inline below current content) --------
    function openCameraLive(id, inline) {
        var mjpegUrl = new URL("/video/" + encodeURIComponent(id) + "/mjpeg", window.location.origin).toString();
        var snapUrl = new URL("/video/" + encodeURIComponent(id) + "/snapshot.jpg", window.location.origin).toString();

        var block =
            "<h4>Live MJPEG</h4>" +
            "<div><img id=\"mjpeg-" + encodeURIComponent(id) + "\" src=\"" + mjpegUrl + "\" alt=\"Live MJPEG\"></div>" +
            "<div class=\"muted\" style=\"margin-top:.25rem\">If MJPEG doesn’t play, a snapshot will appear.</div>";

        var body = document.getElementById("modal-body");
        if (!body) return;

        if (inline) {
            body.insertAdjacentHTML("beforeend", block);
            setTimeout(function () {
                var img = document.getElementById("mjpeg-" + id);
                if (img) {
                    var dims = cameraInfo[id];
                    if (dims && dims.width && dims.height) {
                        applyAspectRatio(img, dims);
                    } else {
                        getCameraDims(id, snapUrl, function (found) { applyAspectRatio(img, found || null); });
                    }
                    img.addEventListener("error", function () {
                        img.src = snapUrl + "?t=" + Date.now();
                    });
                }
            }, 0);
        } else {
            showModal("Camera: " + id, block);
            setTimeout(function () {
                var img2 = document.getElementById("mjpeg-" + id);
                if (img2) {
                    var dims2 = cameraInfo[id];
                    if (dims2 && dims2.width && dims2.height) {
                        applyAspectRatio(img2, dims2);
                    } else {
                        getCameraDims(id, snapUrl, function (found2) { applyAspectRatio(img2, found2 || null); });
                    }
                    img2.addEventListener("error", function () {
                        img2.src = snapUrl + "?t=" + Date.now();
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
        if (mBody) mBody.innerHTML = html || "";
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
