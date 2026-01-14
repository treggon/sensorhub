
/* static/summary.js — full restored version + voxel view (clean DOM operations) */
/* Extended with Navigation (Traverse + Cliff) summary + colorized rays image */
/* Cleaned Actions column: only Details / History / Latest (no Voxel Routes, no Top-Down) */

/* Refresh cadence (ms) — both set to 5000 so decay changes are visible */
const NAV_POLL_MS = 5000;
const SYS_POLL_MS = 5000;

(function () {
  "use strict";

  // ------------------------------
  // State
  // ------------------------------
  var sensorsIndex = {};   // id -> summary record (from /api/summary)
  var cameraInfo = {};   // id -> { width, height } cached from /video/cameras or probe

  // ------------------------------
  // Status helpers + modal helpers
  // ------------------------------
  function statusClass(s) {
    if (s === "ok" || s === true) return "ok";
    if (s === "warning") return "warning";
    if (s === "error" || s === false) return "error";
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

  function showModal(title, initialHtml) {
    var modal = document.getElementById("modal");
    var header = document.getElementById("modal-title");
    var body = document.getElementById("modal-body");
    if (!modal || !header || !body) return;
    header.textContent = title || "";
    body.innerHTML = initialHtml || "";
    modal.style.display = "block";
  }

  function closeModal() {
    var modal = document.getElementById("modal");
    if (modal) modal.style.display = "none";
  }

  // ------------------------------
  // Modal chrome wiring + on-close callbacks
  // ------------------------------
  var __modalCloseCbs = [];   // functions to run when modal closes

  function onModalClose(cb) {
    if (typeof cb === "function") __modalCloseCbs.push(cb);
  }

  function runModalCloseCbs() {
    try {
      for (var i = 0; i < __modalCloseCbs.length; i++) {
        try { __modalCloseCbsi; } catch (e) { /* ignore */ }
      }
    } finally {
      __modalCloseCbs = [];
    }
  }

  var __origCloseModal = closeModal;
  closeModal = function () {
    runModalCloseCbs();
    __origCloseModal();
  };

  function wireModalClose() {
    var modal = document.getElementById("modal");
    var closeBtn = document.getElementById("modal-close");   // <button id="modal-close">×</button>
    var modalInner = document.getElementById("modal-inner");   // inner panel (update ID if different)

    if (closeBtn) {
      closeBtn.onclick = function (e) {
        e.preventDefault();
        closeModal();
      };
    }

    // Click on backdrop closes (if inner panel exists)
    if (modal && modalInner) {
      modal.addEventListener("click", function (e) {
        if (e.target === modal) {
          closeModal();
        }
      });
    }

    // ESC key closes
    document.addEventListener("keydown", function onEsc(ev) {
      if (ev.key === "Escape" || ev.key === "Esc" || ev.keyCode === 27) {
        if (modal && modal.style.display === "block") {
          closeModal();
        }
      }
    });
  }

  // ------------------------------
  // Camera helpers (aspect inference)
  // ------------------------------
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

  // ------------------------------
  // Snapshot insertion (top-of-modal)
  // ------------------------------
  function mountSnapshotAtTop(imgId, titleText, urlCandidates, afterInsertCb) {
    var body = document.getElementById("modal-body");
    if (!body || !urlCandidates || !urlCandidates.length) return;

    var h = document.createElement("h5");
    h.textContent = titleText || "Snapshot";
    body.insertBefore(h, body.firstChild);

    var img = document.createElement("img");
    img.id = imgId;
    img.alt = titleText || "Snapshot";
    img.style.visibility = "hidden";
    img.style.width = "100%";
    img.style.height = "auto";
    img.style.objectFit = "contain";
    img.style.border = "1px solid #333";
    img.style.borderRadius = "6px";
    img.style.maxHeight = "70vh";
    img.style.display = "block";

    if (h.nextSibling) body.insertBefore(img, h.nextSibling);
    else body.appendChild(img);

    var idx = 0;
    img.onload = function () { img.style.visibility = "visible"; };
    img.onerror = function () {
      idx += 1;
      if (idx < urlCandidates.length) {
        img.src = urlCandidates[idx] + "?t=" + Date.now();
      }
    };
    img.src = urlCandidates[0] + "?t=" + Date.now();

    try { if (typeof afterInsertCb === "function") afterInsertCb(img); } catch (_) { }
  }

  // ------------------------------
  // Table render (Actions: only Details / History / Latest)
  // ------------------------------
  function renderSensors(sensors) {
    var tbody = document.querySelector("#sensors-table tbody");
    if (!tbody) return;
    while (tbody.firstChild) tbody.removeChild(tbody.firstChild);

    for (var i = 0; i < sensors.length; i++) {
      var s = sensors[i];
      var tr = document.createElement("tr");

      var tdStatus = document.createElement("td");
      var dot = document.createElement("span");
      dot.className = "status-dot " + statusClass(s.health);
      tdStatus.appendChild(dot);
      tr.appendChild(tdStatus);

      var tdName = document.createElement("td");
      tdName.textContent = s.name || s.id;
      tr.appendChild(tdName);

      var tdType = document.createElement("td");
      tdType.textContent = s.type || s.kind || "-";
      tr.appendChild(tdType);

      var tdActions = document.createElement("td");
      tdActions.className = "actions";

      var btnDetails = document.createElement("button");
      btnDetails.className = "btn";
      btnDetails.textContent = "Details";
      btnDetails.dataset.sensorId = s.id;
      btnDetails.addEventListener("click", function (ev) {
        var id = ev.currentTarget.dataset.sensorId;
        openSensor(id);
      });
      tdActions.appendChild(btnDetails);

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

  // ------------------------------
  // Summary fetch (System + Sensors)
  // ------------------------------
  async function loadSummary() {
    var url = new URL("/api/summary", window.location.origin).toString();
    try {
      var r = await fetch(url, { cache: "no-store" });
      if (!r.ok) { showError("HTTP " + r.status + " " + r.statusText); return; }
      var data = await r.json();
      clearError();

      setDot("dot-health", (data.system && data.system.health) || "unknown");
      setDot("dot-ready", (data.system && data.system.ready) || "unknown");
      setDot("dot-video", (data.system && data.system.video) || "unknown");
      var tsEl = document.getElementById("timestamp");
      var tsVal = (data.system && data.system.timestamp) ? (data.system.timestamp * 1000) : Date.now();
      if (tsEl) tsEl.textContent = "Last updated: " + new Date(tsVal).toLocaleString();

      sensorsIndex = {};
      var sensors = data.sensors || [];
      for (var i = 0; i < sensors.length; i++) sensorsIndex[sensors[i].id] = sensors[i];

      try {
        var camsResp = await fetch(new URL("/video/cameras", window.location.origin).toString(), { cache: "no-store" });
        if (camsResp.ok) {
          var cams = await camsResp.json();
          var list = Array.isArray(cams) ? cams : ((cams && cams.cameras) ? cams.cameras : []);
          for (var j = 0; j < list.length; j++) {
            var c = list[j] || {};
            var cid = c.id || c.camera_id || c.name;
            var w = c.width || c.w || c.frame_width;
            var h = c.height || c.h || c.frame_height;
            if (cid && w && h) cameraInfo[cid] = { width: Number(w), height: Number(h) };
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

  // ------------------------------
  // Voxel helpers (scoped to livox_voxel)
  // ------------------------------
  function insertTopdownPNGAtTop(titleText, pngUrl) {
    var body = document.getElementById("modal-body");
    if (!body) return;

    var h = document.createElement("h5");
    h.textContent = titleText || "Top-down";
    body.insertBefore(h, body.firstChild);

    var img = document.createElement("img");
    img.id = "voxel-topdown";
    img.src = pngUrl + "&t=" + Date.now();
    img.alt = "Top-down occupancy";
    img.style.width = "100%";
    img.style.height = "auto";
    img.style.objectFit = "contain";
    img.style.border = "1px solid #333";
    img.style.borderRadius = "6px";
    img.style.maxHeight = "70vh";
    img.style.display = "block";
    body.insertBefore(img, h.nextSibling);

    var row = document.createElement("div");
    row.id = "traverse-row";
    row.style.display = "flex";
    row.style.alignItems = "center";
    row.style.gap = "10px";
    row.style.margin = "8px 0 18px 0";

    var dot = document.createElement("span");
    dot.id = "traverse-dot";
    dot.className = "status-dot unknown";
    row.appendChild(dot);

    var txt = document.createElement("span");
    txt.id = "traverse-text";
    txt.style.fontWeight = "500";
    txt.textContent = "Traversability: checking…";
    row.appendChild(txt);

    body.insertBefore(row, img.nextSibling);
  }

  async function refreshTraversability(signalUrl) {
    var dot = document.getElementById("traverse-dot");
    var txt = document.getElementById("traverse-text");
    try {
      var r = await fetch(new URL(signalUrl, window.location.origin).toString(), { cache: "no-store" });
      if (!r.ok) {
        if (dot) dot.className = "status-dot warning";
        if (txt) txt.textContent = "Traversability: HTTP " + r.status + " " + r.statusText;
        return;
      }
      var j = await r.json();
      var ok = (j.ok !== undefined) ? !!j.ok : !!j.ok_traverse;
      var pitch = (j.pitch_deg == null) ? "n/a" : String(j.pitch_deg) + "°";
      var step = (j.max_step_m == null) ? "n/a" : String(j.max_step_m) + " m";
      var climb = j.climb_limit_deg != null ? j.climb_limit_deg : (j.slope && j.slope.climb_limit_deg);

      if (dot) dot.className = "status-dot " + (ok ? "ok" : "error");
      if (txt) {
        txt.textContent = "Traversability: " + (ok ? "OK" : "NOT OK") +
          "  |  pitch " + pitch + (climb != null ? " (≤ " + climb + "°)" : "") +
          "  |  step " + step + (j.step_limit_m != null ? " (≤ " + j.step_limit_m + " m)" : "");
      }
    } catch (e) {
      if (dot) dot.className = "status-dot warning";
      if (txt) txt.textContent = "Traversability: check failed";
    }
  }

  // ------------------------------
  // Open sensor modal (restored + voxel)
  // ------------------------------
  async function openSensor(id) {
    showModal("Sensor: " + id, "");
    var body = document.getElementById("modal-body");
    var s = sensorsIndex[id] || {};

    var kindStr = String(s.type || s.kind || "").toLowerCase();
    var isCamera = (s.type === "camera" || s.kind === "camera" || kindStr === "camera");
    var isLivoxVoxel = (id === "livox_voxel" || kindStr === "voxelgrid");
    var isLivox = (!isLivoxVoxel && (id === "livox" || kindStr.indexOf("livox") >= 0));
    var isRplidar = (id.indexOf("rplidar") >= 0 || kindStr.indexOf("lidar2d") >= 0);

    // 1) Snapshot / top image
    if (isLivoxVoxel) {
      var params = "scale_mode=auto&cmap=gray&draw_grid=0&crop_radius_m=10&downscale=2&mark_center=1";
      insertTopdownPNGAtTop("Top-down Occupancy (Strength)", "/livox_voxel/topdown.png?" + params);
      refreshTraversability("/livox_voxel/traverse/check");
      var _pngTimer = setInterval(function () {
        var img = document.getElementById("voxel-topdown");
        if (img) img.src = "/livox_voxel/topdown.png?" + params + "&t=" + Date.now();
        refreshTraversability("/livox_voxel/traverse/check");
      }, 2000);
      onModalClose(function () { clearInterval(_pngTimer); });
    } else if (isCamera) {
      var snap = new URL("/video/" + encodeURIComponent(id) + "/snapshot.jpg", window.location.origin).toString();
      mountSnapshotAtTop("cam-snap-" + encodeURIComponent(id), "Camera Snapshot", [snap], function (img) {
        var dims = cameraInfo[id];
        if (dims && dims.width && dims.height) {
          applyAspectRatio(img, dims);
        } else {
          getCameraDims(id, snap, function (found) { applyAspectRatio(img, found || null); });
        }
      });
      if (body) {
        var liveBtn = document.createElement("button");
        liveBtn.className = "btn";
        liveBtn.id = "btn-live-" + id;
        liveBtn.textContent = "Open Live MJPEG";
        liveBtn.addEventListener("click", function () { openCameraLive(id, true); });
        body.appendChild(liveBtn);
      }
    } else if (isRplidar) {
      var rplidarCandidates = [
        new URL("/sensors/" + encodeURIComponent(id) + "/snapshot.png", window.location.origin).toString(),
        new URL("/sensors/" + encodeURIComponent(id) + "/snapshot.jpg", window.location.origin).toString()
      ];
      mountSnapshotAtTop("snap-" + encodeURIComponent(id), "RPLidar Snapshot", rplidarCandidates, function (img) {
        img.style.width = "100%";
        img.style.height = "auto";
        img.style.objectFit = "contain";
        img.style.border = "1px solid #333";
        img.style.borderRadius = "6px";
        img.style.maxHeight = "70vh";
        img.style.display = "block";
      });
    } else if (isLivox) {
      var livoxCandidates = [
        new URL("/sensors/" + encodeURIComponent(id) + "/snapshot.png", window.location.origin).toString(),
        new URL("/sensors/" + encodeURIComponent(id) + "/snapshot.jpg", window.location.origin).toString(),
        new URL("/livox/snapshot.png", window.location.origin).toString(),
        new URL("/livox/snapshot.jpg", window.location.origin).toString()
      ];
      mountSnapshotAtTop("livox-snap-" + encodeURIComponent(id), "Livox Snapshot", livoxCandidates, function (img) {
        img.style.width = "100%";
        img.style.height = "auto";
        img.style.objectFit = "contain";
        img.style.border = "1px solid #333";
        img.style.borderRadius = "6px";
        img.style.maxHeight = "70vh";
        img.style.display = "block";
      });
    } else {
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

    // 2) Health
    try {
      var hURL = new URL("/sensors/" + encodeURIComponent(id) + "/health", window.location.origin).toString();
      var hResp = await fetch(hURL, { cache: "no-store" });
      var hTitle = document.createElement("h5");
      hTitle.textContent = "Health";
      body.appendChild(hTitle);

      var healthBlock = document.createElement("pre");
      if (hResp.ok) {
        var hJson = await hResp.json();
        healthBlock.textContent = JSON.stringify(hJson, null, 2);
      } else {
        healthBlock.textContent = "HTTP " + hResp.status + " " + hResp.statusText;
      }
      body.appendChild(healthBlock);
    } catch (e) {
      console.error("Health fetch failed:", e);
      var hErr = document.createElement("pre");
      hErr.textContent = "Health fetch failed: " + String((e && e.message) || e);
      body.appendChild(hErr);
    }

    // 3) Latest (metadata only)
    try {
      var latestUrl = "/sensors/" + encodeURIComponent(id) + "/latest";
      if (latestUrl.indexOf("?") === -1) latestUrl += "?";
      if (!/include_meta=/.test(latestUrl)) latestUrl += (latestUrl.endsWith("?") ? "" : "&") + "include_meta=true";
      if (!/include_points=/.test(latestUrl)) latestUrl += "&include_points=false";
      var lURL = new URL(latestUrl, window.location.origin).toString();
      var lResp = await fetch(lURL, { cache: "no-store" });

      var lTitle = document.createElement("h5");
      lTitle.textContent = "Latest";
      body.appendChild(lTitle);

      var latestBlock = document.createElement("pre");
      if (lResp.ok) {
        var lJson = await lResp.json();
        latestBlock.textContent = JSON.stringify(lJson, null, 2);
      } else if (lResp.status === 404) {
        latestBlock.textContent = "No sample yet (404)";
      } else if (lResp.status === 204) {
        latestBlock.textContent = "No content (204)";
      } else {
        latestBlock.textContent = "HTTP " + lResp.status + " " + lResp.statusText;
      }
      body.appendChild(latestBlock);
    } catch (e) {
      console.error("Latest fetch failed:", e);
      var lErr = document.createElement("pre");
      lErr.textContent = "Latest fetch failed: " + String((e && e.message) || e);
      body.appendChild(lErr);
    }

    // 4) ACTION LINKS (no voxel/top-down buttons)
    if (body) {
      var actionsWrap = document.createElement("div");
      actionsWrap.style.marginTop = "8px";

      var linkRaw = document.createElement("a");
      linkRaw.className = "btn";
      linkRaw.textContent = "Latest Raw";
      linkRaw.href = "/sensors/" + encodeURIComponent(id) + "/latest_raw";
      linkRaw.target = "_blank";

      var linkHist = document.createElement("a");
      linkHist.className = "btn";
      linkHist.textContent = "History";
      linkHist.href = "/sensors/" + encodeURIComponent(id) + "/history";
      linkHist.target = "_blank";

      actionsWrap.appendChild(linkRaw);
      actionsWrap.appendChild(linkHist);
      body.appendChild(actionsWrap);
    }
  }

  // ------------------------------
  // Live camera view (DOM-safe)
  // ------------------------------
  function openCameraLive(id, inline) {
    var mjpegUrl = new URL("/video/" + encodeURIComponent(id) + "/mjpeg", window.location.origin).toString();
    var snapUrl = new URL("/video/" + encodeURIComponent(id) + "/snapshot.jpg", window.location.origin).toString();

    var body = document.getElementById("modal-body");
    if (!body) return;

    var h = document.createElement("h5");
    h.textContent = "Live MJPEG";
    body.appendChild(h);

    var img = document.createElement("img");
    img.src = mjpegUrl;
    img.alt = "Live MJPEG";
    img.style.width = "100%";
    img.style.height = "auto";
    img.style.objectFit = "contain";
    img.style.border = "1px solid #333";
    img.style.borderRadius = "6px";
    img.style.maxHeight = "70vh";
    img.style.display = "block";
    body.appendChild(img);

    var p = document.createElement("p");
    var a = document.createElement("a");
    a.className = "btn";
    a.href = snapUrl;
    a.target = "_blank";
    a.textContent = "Open Snapshot";
    p.appendChild(a);
    body.appendChild(p);
  }

  // ------------------------------
  // Navigation summary (Traverse + Cliff) + colorized rays image
  // ------------------------------
  async function loadNavigation() {
    try {
      var url = new URL("/livox_voxel/traverse/summary", window.location.origin);
      url.searchParams.set("ahead_m", "1.0");
      url.searchParams.set("width_m", "1.0");
      url.searchParams.set("forward_only", "1");
      url.searchParams.set("window", "1");
      url.searchParams.set("z_window", "1");
      url.searchParams.set("method", "plane");
      url.searchParams.set("rays_n", "16");
      url.searchParams.set("rays_steps", "8");
      url.searchParams.set("cliff_threshold_deg", "45");

      var r = await fetch(url.toString(), { cache: "no-store" });
      if (!r.ok) throw new Error("HTTP " + r.status + " " + r.statusText);
      var data = await r.json();

      setDot("dot-traverse", data.ok_traverse);
      setDot("dot-cliff", data.ok_cliff);

      var p = (typeof data.pitch_deg === "number") ? data.pitch_deg.toFixed(2) : "—";
      var step = (typeof data.max_step_m === "number") ? data.max_step_m.toFixed(3) : "—";
      var clMax = (data.cliff && typeof data.cliff.max_deg === "number") ? data.cliff.max_deg.toFixed(2) : "—";
      var clDir = (data.cliff && typeof data.cliff.dir_deg === "number") ? data.cliff.dir_deg.toFixed(1) : "—";
      var line = "Traverse: " + (data.ok_traverse ? "OK" : "NOT OK") +
        " | pitch " + p + "° (≤" + data.climb_limit_deg + "°)" +
        " | step " + step + " m (≤" + data.step_limit_m + " m)" +
        "  ·  Cliff: " + (data.ok_cliff ? "OK" : "NOT OK") +
        " | max " + clMax + "° @ " + clDir + "°";
      var navSummary = document.getElementById("navSummary");
      if (navSummary) navSummary.textContent = line;

      var imgEl = document.getElementById("navImage");
      if (imgEl && data.image && data.image.url) {
        var imgUrl = new URL(data.image.url, window.location.origin).toString() + "&t=" + Date.now();
        imgEl.src = imgUrl;
        imgEl.alt = "Top-down with directional rays";
      }

      var navNote = document.getElementById("navNote");
      if (navNote) {
        if (data.status !== "ok") {
          navNote.textContent = "Navigation summary failed: " + (data.error || "No data");
        } else {
          var w = data.window || {};
          navNote.textContent = "Window: XY=" + w.xy + " Z=" + w.z + " · forward_only=" + w.forward_only;
        }
      }
    } catch (e) {
      console.warn("Navigation fetch failed:", e);
      setDot("dot-traverse", "unknown");
      setDot("dot-cliff", "unknown");
      var navSummary = document.getElementById("navSummary");
      if (navSummary) navSummary.textContent = "Navigation summary failed. Check service.";
    }
  }

  // ------------------------------
  // Boot
  // ------------------------------
  window.addEventListener("DOMContentLoaded", function () {
    wireModalClose();
    loadSummary();        // system + sensors
    loadNavigation();     // traverse + cliff + image
    setInterval(loadNavigation, NAV_POLL_MS);
    setInterval(loadSummary, SYS_POLL_MS);
  });
})();
