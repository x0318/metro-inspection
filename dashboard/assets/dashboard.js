const { createApp, ref, computed, watch, nextTick, onMounted, onBeforeUnmount } = Vue;

createApp({
  setup() {
    const activeView = ref('overview');
    const records = ref([]);
    const backendOnline = ref(false);
    const lastUpdated = ref(null);
    const errorMessage = ref('');
    const refreshing = ref(false);
    const selectedCamera = ref('all');
    const selectedSeverity = ref('all');
    const searchText = ref('');
    const selectedId = ref('');
    const cameraStatuses = ref([]);
    const previewProfile = ref({
      max_width: 640,
      max_height: 360,
      jpeg_quality: 55,
      default_fps: 6,
      max_fps: 10
    });
    const focusedCameraId = ref('');
    const selectedMonitorCameraId = ref('xj1');
    const compactCameraLayout = ref(false);
    let pollTimer = null;
    let cameraMediaQuery = null;
    let severityChart = null;
    let typeChart = null;

    const cameraCatalog = Object.freeze([
      { id: 'xj1', label: 'XJ1' },
      { id: 'xj2', label: 'XJ2' },
      { id: 'xj3', label: 'XJ3' },
      { id: 'xj4', label: 'XJ4' },
      { id: 'pitch_camera', label: 'Pitch' }
    ]);
    const cameras = Object.freeze([
      { id: 'all', label: '全部相机' },
      ...cameraCatalog
    ]);

    const monitorCameras = computed(() => cameraCatalog.map(camera => {
      const status = cameraStatuses.value.find(item => item.id === camera.id);
      return {
        ...camera,
        topic: status?.topic || '--',
        available: Boolean(status?.available),
        source_fps: Number(status?.source_fps || 0),
        age_seconds: status?.age_seconds ?? null,
        frames_received: Number(status?.frames_received || 0),
        stream_url: status?.stream_url || `/api/cameras/${camera.id}/stream.mjpg`
      };
    }));

    const onlineCameraCount = computed(() => (
      monitorCameras.value.filter(camera => camera.available).length
    ));

    const focusedCamera = computed(() => (
      monitorCameras.value.find(camera => camera.id === focusedCameraId.value) || null
    ));

    const selectedMonitorCamera = computed(() => (
      monitorCameras.value.find(camera => camera.id === selectedMonitorCameraId.value)
      || monitorCameras.value[0]
      || null
    ));

    function normalizeCameraName(value) {
      return value === 'pitch' ? 'pitch_camera' : String(value || '').trim();
    }

    function cameraLabel(value) {
      const normalized = normalizeCameraName(value);
      const camera = cameras.find(item => item.id === normalized);
      return camera ? camera.label : (normalized || '--');
    }

    function cameraCount(cameraId) {
      if (cameraId === 'all') return records.value.length;
      return records.value.filter(item => normalizeCameraName(item.camera_name) === cameraId).length;
    }

    const filteredRecords = computed(() => {
      const query = searchText.value.toLocaleLowerCase('zh-CN');
      return records.value.filter(item => {
        const cameraMatches = selectedCamera.value === 'all'
          || normalizeCameraName(item.camera_name) === selectedCamera.value;
        const severityMatches = selectedSeverity.value === 'all'
          || String(Number(item.severity || 0)) === selectedSeverity.value;
        const haystack = [
          item.event_id,
          item.detection_id,
          item.type,
          item.class_name,
          item.mileage,
          item.semantic_location?.segment_name,
          item.semantic_location?.segment_id
        ].filter(value => value !== null && value !== undefined).join(' ').toLocaleLowerCase('zh-CN');
        return cameraMatches && severityMatches && (!query || haystack.includes(query));
      });
    });

    const selectedRecord = computed(() => (
      filteredRecords.value.find(item => item.event_id === selectedId.value) || null
    ));

    const summary = computed(() => ({
      total: records.value.length,
      localized: records.value.filter(item => item.has_3d_position).length,
      semantic: records.value.filter(item => item.has_semantic_location).length,
      severe: records.value.filter(item => Number(item.severity) === 3).length,
      unknown: records.value.filter(item => Number(item.severity || 0) === 0).length
    }));

    const lastUpdatedText = computed(() => {
      if (!lastUpdated.value) return '尚未同步';
      return lastUpdated.value.toLocaleTimeString('zh-CN', { hour12: false });
    });

    watch(filteredRecords, list => {
      if (!list.some(item => item.event_id === selectedId.value)) {
        selectedId.value = list.length ? list[0].event_id : '';
      }
    }, { immediate: true });

    watch(activeView, value => {
      if (value === 'report') nextTick(renderCharts);
      if (value !== 'cameras') focusedCameraId.value = '';
    });

    watch(records, () => {
      if (activeView.value === 'report') nextTick(renderCharts);
    });

    async function fetchJson(path) {
      const response = await fetch(path, { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return response.json();
    }

    async function refreshData() {
      if (refreshing.value) return;
      refreshing.value = true;
      try {
        const [health, defects, cameraData] = await Promise.all([
          fetchJson('/api/health'),
          fetchJson('/api/defects'),
          fetchJson('/api/cameras')
        ]);
        if (health.service !== 'metro_dashboard_bridge' || health.status !== 'online') {
          throw new Error('服务响应不符合平台接口');
        }
        records.value = Array.isArray(defects.records) ? defects.records : [];
        cameraStatuses.value = Array.isArray(cameraData.cameras) ? cameraData.cameras : [];
        if (cameraData.preview && typeof cameraData.preview === 'object') {
          previewProfile.value = { ...previewProfile.value, ...cameraData.preview };
        }
        backendOnline.value = true;
        errorMessage.value = '';
        lastUpdated.value = new Date();
      } catch (error) {
        backendOnline.value = false;
        cameraStatuses.value = [];
        errorMessage.value = error instanceof Error ? error.message : '无法读取病害接口';
      } finally {
        refreshing.value = false;
      }
    }

    function levelText(item) {
      return item?.level || '未评估';
    }

    function severityClass(item) {
      const severity = Math.max(0, Math.min(3, Number(item?.severity || 0)));
      return `severity severity-${severity}`;
    }

    function confidenceText(value) {
      if (value === null || value === undefined || value === '') return '--';
      const number = Number(value);
      return Number.isFinite(number) ? `${(number * 100).toFixed(1)}%` : '--';
    }

    function formatNumber(value, digits = 2) {
      const number = Number(value);
      return Number.isFinite(number) ? number.toFixed(digits) : '--';
    }

    function formatTime(value) {
      if (!value) return '--';
      const date = new Date(value);
      if (Number.isNaN(date.valueOf())) return '--';
      return date.toLocaleString('zh-CN', { hour12: false });
    }

    function bboxText(bbox) {
      if (!bbox) return '--';
      return `x ${formatNumber(bbox.x, 0)}, y ${formatNumber(bbox.y, 0)}, ${formatNumber(bbox.width, 0)} x ${formatNumber(bbox.height, 0)} px`;
    }

    function positionText(position) {
      if (!position) return '--';
      return `X ${formatNumber(position.x)} / Y ${formatNumber(position.y)} / Z ${formatNumber(position.z)} m`;
    }

    function stageText(item) {
      if (item?.has_semantic_location) return '工程定位完成';
      if (item?.has_3d_position) return '三维定位完成';
      return '二维识别';
    }

    function localizationText(item) {
      const labels = {
        none: '未定位',
        current_cloud: '当前点云',
        accumulated_map: '积累点云地图',
        tunnel_model: '隧道模型求交'
      };
      return labels[item?.localization?.method_name] || item?.localization?.method_name || '--';
    }

    function segmentText(item) {
      const semantic = item?.semantic_location;
      if (!semantic) return '--';
      const name = semantic.segment_name || '区段';
      return `${name} ${semantic.segment_id}`;
    }

    function clockText(value) {
      if (value === null || value === undefined || value === '') return '--';
      return `${formatNumber(value, 1)} 点钟`;
    }

    function locationText(item) {
      const mileage = item?.mileage || '--';
      const segment = segmentText(item);
      return segment === '--' ? mileage : `${mileage} / ${segment}`;
    }

    function formatFps(value) {
      const fps = Number(value);
      if (!Number.isFinite(fps) || fps <= 0) return '0';
      return Number.isInteger(fps) ? String(fps) : fps.toFixed(1);
    }

    function cameraFpsText(camera) {
      return `${formatFps(camera?.source_fps)} FPS`;
    }

    function cameraAgeText(camera) {
      if (camera?.age_seconds === null || camera?.age_seconds === undefined) return '尚未收到帧';
      const age = Number(camera.age_seconds);
      if (!Number.isFinite(age)) return '尚未收到帧';
      return age < 0.1 ? '刚刚更新' : `${age.toFixed(1)} 秒前`;
    }

    function streamUrl(camera) {
      const separator = camera.stream_url.includes('?') ? '&' : '?';
      return `${camera.stream_url}${separator}fps=${previewProfile.value.default_fps}`;
    }

    function updateCameraLayout(event) {
      compactCameraLayout.value = event.matches;
      if (event.matches) focusedCameraId.value = '';
    }

    function renderCharts() {
      const severityElement = document.getElementById('severity-chart');
      const typeElement = document.getElementById('type-chart');
      if (!severityElement || !typeElement || typeof echarts === 'undefined') return;
      severityChart = severityChart || echarts.init(severityElement);
      typeChart = typeChart || echarts.init(typeElement);

      const severityCounts = [0, 1, 2, 3].map(level => (
        records.value.filter(item => Number(item.severity || 0) === level).length
      ));
      severityChart.setOption({
        animationDuration: 250,
        tooltip: { trigger: 'axis' },
        grid: { left: 42, right: 14, top: 24, bottom: 34 },
        xAxis: { type: 'category', data: ['未评估', 'I级', 'II级', 'III级'] },
        yAxis: { type: 'value', minInterval: 1 },
        series: [{
          type: 'bar', barMaxWidth: 46,
          data: severityCounts.map((value, index) => ({
            value,
            itemStyle: { color: ['#7b8790', '#2463a7', '#c47a00', '#b42318'][index] }
          }))
        }]
      });

      const counts = {};
      records.value.forEach(item => {
        const name = item.type || '未分类';
        counts[name] = (counts[name] || 0) + 1;
      });
      const typeData = Object.entries(counts).map(([name, value]) => ({ name, value }));
      typeChart.setOption({
        animationDuration: 250,
        tooltip: { trigger: 'item' },
        legend: { bottom: 0, type: 'scroll' },
        color: ['#147a5b', '#2463a7', '#c47a00', '#8a4f9e', '#b42318', '#546e7a'],
        series: [{
          type: 'pie', radius: ['38%', '66%'], center: ['50%', '43%'],
          label: { formatter: '{b}: {c}' },
          data: typeData.length ? typeData : [{ name: '暂无数据', value: 0, itemStyle: { color: '#d8dee3' } }]
        }]
      });
    }

    function resizeCharts() {
      if (severityChart) severityChart.resize();
      if (typeChart) typeChart.resize();
    }

    function printReport() {
      window.print();
    }

    onMounted(() => {
      cameraMediaQuery = window.matchMedia('(max-width: 720px)');
      updateCameraLayout(cameraMediaQuery);
      cameraMediaQuery.addEventListener('change', updateCameraLayout);
      refreshData();
      pollTimer = window.setInterval(refreshData, 2000);
      window.addEventListener('resize', resizeCharts);
    });

    onBeforeUnmount(() => {
      if (pollTimer) window.clearInterval(pollTimer);
      if (severityChart) severityChart.dispose();
      if (typeChart) typeChart.dispose();
      if (cameraMediaQuery) cameraMediaQuery.removeEventListener('change', updateCameraLayout);
      window.removeEventListener('resize', resizeCharts);
    });

    return {
      activeView,
      records,
      backendOnline,
      lastUpdatedText,
      errorMessage,
      refreshing,
      selectedCamera,
      selectedSeverity,
      searchText,
      selectedId,
      previewProfile,
      focusedCameraId,
      selectedMonitorCameraId,
      compactCameraLayout,
      cameras,
      monitorCameras,
      onlineCameraCount,
      focusedCamera,
      selectedMonitorCamera,
      filteredRecords,
      selectedRecord,
      summary,
      cameraLabel,
      cameraCount,
      refreshData,
      levelText,
      severityClass,
      confidenceText,
      formatTime,
      bboxText,
      positionText,
      stageText,
      localizationText,
      segmentText,
      clockText,
      locationText,
      formatFps,
      cameraFpsText,
      cameraAgeText,
      streamUrl,
      printReport
    };
  }
}).mount('#app');
