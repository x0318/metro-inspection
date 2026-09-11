const { createApp, ref, computed, onMounted, onBeforeUnmount, watch, nextTick } = Vue;

createApp({
  setup() {
    const activeTab = ref('monitor');
    const status = ref(defaultStatus());
    const options = ref({ world: '', modes: [], destinations: [], junction: { choices: [] } });
    const selectedMode = ref('junction');
    const selectedDestination = ref('main_end');
    const selectedCamera = ref('xj1');
    const defectList = ref([]);
    const sessionToken = ref('');
    const toast = ref({ message: '', type: '' });
    const backendOnline = ref(false);
    let socket = null;
    let socketReconnect = null;
    let heartbeatTimer = null;
    let fallbackPollTimer = null;
    let defectPollTimer = null;
    let toastTimer = null;
    let levelChart = null;
    let typeChart = null;
    let sessionRefreshPromise = null;

    const cameras = ref([]);

    function defaultStatus() {
      return {
        ros: { online: false, external_node_count: 0 },
        odom: { fresh: false, pose: { x: null, y: null, yaw: null }, velocity: { linear: 0, angular: 0 }, mileage_m: 0 },
        navigation: { state: 'idle', progress_percent: 0, poses_remaining: 0, can_pause: false, can_resume: false, can_cancel: false, message: '等待平台连接' },
        safety: { estop_active: false, message: '安全许可状态未知' },
        cameras: {}
      };
    }

    const rosOnline = computed(() => status.value.ros?.online === true);
    const odom = computed(() => status.value.odom || defaultStatus().odom);
    const navigation = computed(() => status.value.navigation || defaultStatus().navigation);
    const safety = computed(() => status.value.safety || defaultStatus().safety);
    const stateText = computed(() => ({
      idle: '系统待机', starting: '任务启动中', running: '巡检中', waiting_choice: '等待岔路选择',
      paused: '任务已暂停', canceling: '正在取消', completed: '巡检完成', failed: '导航失败', estop: '急停锁定'
    }[navigation.value.state] || '状态未知'));
    const stateDotClass = computed(() => navigation.value.state || 'idle');
    const backendTitle = computed(() => backendOnline.value ? 'FastAPI 与状态推送正常' : '无法连接平台后端');
    const rosTitle = computed(() => rosOnline.value
      ? `发现 ${status.value.ros.external_node_count || 0} 个外部 ROS 2 节点，/odom ${odom.value.fresh ? '正常' : '无新数据'}`
      : '未发现平台桥之外的 ROS 2 节点');
    const progressWidth = computed(() => `${Math.min(100, Math.max(0, Number(navigation.value.progress_percent || 0)))}%`);
    const taskLocked = computed(() => ['starting', 'running', 'waiting_choice', 'paused', 'canceling'].includes(navigation.value.state));
    const canStart = computed(() => sessionToken.value && !taskLocked.value && !safety.value.estop_active && rosOnline.value && odom.value.fresh);
    const onlineCameraCount = computed(() => cameras.value.reduce(
      (count, camera) => count + (cameraAvailable(camera.id) ? 1 : 0),
      0
    ));

    function cameraAvailable(cameraName) {
      return status.value.cameras?.[cameraName]?.available === true;
    }

    function cameraStreamUrl(cameraName) {
      return `/api/camera/${cameraName}.mjpeg`;
    }

    function cameraBBox(cameraName) {
      const record = defectList.value.find(item => item.camera_name === cameraName && item.bbox);
      if (!record) return null;
      const box = record.bbox;
      const left = box.x / box.image_width * 100;
      const top = box.y / box.image_height * 100;
      const width = Math.min(box.width / box.image_width * 100, 100 - left);
      const height = Math.min(box.height / box.image_height * 100, 100 - top);
      return { label: record.type, style: { left: `${left}%`, top: `${top}%`, width: `${width}%`, height: `${height}%` } };
    }

    function selectCamera(cameraName) {
      selectedCamera.value = cameraName;
    }

    const reportStats = computed(() => {
      const stats = { total: defectList.value.length, level1: 0, level2: 0, level3: 0 };
      defectList.value.forEach(item => {
        if (item.level.startsWith('I级 ')) stats.level1 += 1;
        else if (item.level.startsWith('II级 ')) stats.level2 += 1;
        else if (item.level.startsWith('III级 ')) stats.level3 += 1;
      });
      return stats;
    });

    function showToast(message, type = '') {
      toast.value = { message, type };
      if (toastTimer) clearTimeout(toastTimer);
      toastTimer = setTimeout(() => { toast.value = { message: '', type: '' }; }, 3500);
    }

    async function fetchJson(url, init = {}) {
      const response = await fetch(url, { cache: 'no-store', ...init });
      if (!response.ok) {
        let detail = `${response.status}`;
        try { detail = (await response.json()).detail || detail; } catch (_) {}
        const error = new Error(detail);
        error.status = response.status;
        throw error;
      }
      return response.json();
    }

    async function refreshSessionToken() {
      if (!sessionRefreshPromise) {
        sessionRefreshPromise = fetchJson('/api/session')
          .then(session => {
            if (!session?.token) throw new Error('平台未返回控制会话令牌');
            sessionToken.value = session.token;
            return session.token;
          })
          .finally(() => { sessionRefreshPromise = null; });
      }
      return sessionRefreshPromise;
    }

    async function post(url, payload, retryInvalidSession = true) {
      if (!sessionToken.value) await refreshSessionToken();
      const headers = { 'X-Session-Token': sessionToken.value };
      const init = { method: 'POST', headers };
      if (payload !== undefined) {
        headers['Content-Type'] = 'application/json';
        init.body = JSON.stringify(payload);
      }
      try {
        return await fetchJson(url, init);
      } catch (error) {
        if (retryInvalidSession && error.status === 403 && error.message === 'invalid session token') {
          await refreshSessionToken();
          return post(url, payload, false);
        }
        throw error;
      }
    }

    async function startNavigation() {
      try {
        await post('/api/navigation/start', {
          mode: selectedMode.value,
          destination: selectedMode.value === 'destination' ? selectedDestination.value : null
        });
        showToast('导航任务已提交', 'success');
      } catch (error) { showToast(`启动失败：${error.message}`, 'error'); }
    }

    async function chooseJunction(choice) {
      try {
        await post('/api/navigation/junction-choice', { choice });
        showToast('岔路方向已提交', 'success');
      } catch (error) { showToast(`选择失败：${error.message}`, 'error'); }
    }

    const commandEndpoints = Object.freeze({
      pause: '/api/navigation/pause', resume: '/api/navigation/resume', cancel: '/api/navigation/cancel',
      estop: '/api/safety/estop', reset: '/api/safety/reset'
    });

    async function sendCommand(command) {
      try {
        await post(commandEndpoints[command]);
        showToast(command === 'estop' ? '急停指令已发送' : '控制指令已发送', command === 'estop' ? 'error' : 'success');
      } catch (error) { showToast(`指令失败：${error.message}`, 'error'); }
    }

    function applyStatus(data) {
      if (data?.service !== 'metro_navigation_demo') return;
      status.value = data;
      backendOnline.value = data.backend?.online === true;
    }

    async function pollStatus() {
      try { applyStatus(await fetchJson('/api/status')); }
      catch (_) { backendOnline.value = false; }
    }

    function connectSocket() {
      if (socket) socket.close();
      const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
      socket = new WebSocket(`${protocol}//${location.host}/ws/status`);
      socket.onmessage = event => {
        try { applyStatus(JSON.parse(event.data)); } catch (_) {}
      };
      socket.onopen = () => { backendOnline.value = true; };
      socket.onclose = () => {
        backendOnline.value = false;
        socket = null;
        if (!socketReconnect) socketReconnect = setTimeout(() => { socketReconnect = null; connectSocket(); }, 1500);
      };
    }

    async function pollDefects() {
      try { defectList.value = (await fetchJson('/api/defects')).records || []; }
      catch (_) {}
    }

    async function initialize() {
      try {
        const [, routeOptions, cameraOptions] = await Promise.all([
          refreshSessionToken(), fetchJson('/api/navigation/options'), fetchJson('/api/cameras')
        ]);
        options.value = routeOptions;
        cameras.value = cameraOptions.cameras;
        selectedCamera.value = cameras.value[0].id;
        if (routeOptions.destinations?.length) selectedDestination.value = routeOptions.destinations[0].id;
        await post('/api/camera/select', { camera_name: selectedCamera.value });
      } catch (error) {
        showToast(`平台初始化失败：${error.message}`, 'error');
      }
      await Promise.all([pollStatus(), pollDefects()]);
      connectSocket();
      heartbeatTimer = setInterval(async () => {
        if (!sessionToken.value) return;
        try { await post('/api/session/heartbeat'); } catch (_) { backendOnline.value = false; }
      }, 1000);
      fallbackPollTimer = setInterval(() => { if (!socket || socket.readyState !== WebSocket.OPEN) pollStatus(); }, 1500);
      defectPollTimer = setInterval(pollDefects, 2500);
    }

    function formatNumber(value, digits = 0) {
      const number = Number(value);
      return Number.isFinite(number) ? number.toFixed(digits) : '--';
    }
    function formatMetric(value, unit) {
      if (value === null || value === undefined || value === '') return '--';
      const number = Number(value);
      return Number.isFinite(number) ? `${number.toFixed(2)} ${unit}` : '--';
    }
    function formatTime(value) {
      if (!value) return '--';
      const date = new Date(value);
      return Number.isNaN(date.valueOf()) ? '--' : date.toLocaleString('zh-CN');
    }
    function positionText(position) {
      if (!position) return '--';
      return `X ${formatNumber(position.x, 2)} / Y ${formatNumber(position.y, 2)} / Z ${formatNumber(position.z, 2)}`;
    }
    function levelClass(level) {
      if (level.startsWith('III级')) return 'level level-3';
      if (level.startsWith('II级')) return 'level level-2';
      return 'level level-1';
    }
    function cameraSnapshotUrl(camera, id) { return `/api/camera/${camera}.jpg?v=${encodeURIComponent(id)}`; }
    function generateReport() { window.print(); }

    function renderCharts() {
      const levelElement = document.getElementById('levelChart');
      const typeElement = document.getElementById('typeChart');
      if (!levelElement || !typeElement) return;
      levelChart = levelChart || echarts.init(levelElement);
      typeChart = typeChart || echarts.init(typeElement);
      const stats = reportStats.value;
      levelChart.setOption({
        tooltip: { trigger: 'axis' }, grid: { left: 42, right: 18, top: 28, bottom: 36 },
        xAxis: { type: 'category', data: ['I级', 'II级', 'III级'] }, yAxis: { type: 'value', minInterval: 1 },
        series: [{ type: 'bar', barWidth: 42, data: [
          { value: stats.level1, itemStyle: { color: '#175cd3' } },
          { value: stats.level2, itemStyle: { color: '#e9a100' } },
          { value: stats.level3, itemStyle: { color: '#d92d20' } }
        ] }]
      });
      const counts = {};
      defectList.value.forEach(item => { counts[item.type] = (counts[item.type] || 0) + 1; });
      const typeData = Object.entries(counts).map(([name, value]) => ({ name, value }));
      typeChart.setOption({
        tooltip: { trigger: 'item' }, legend: { bottom: 0, type: 'scroll' },
        series: [{ type: 'pie', radius: ['35%', '62%'], center: ['50%', '44%'],
          label: { formatter: '{b}: {c}' }, data: typeData.length ? typeData : [{ name: '暂无数据', value: 0, itemStyle: { color: '#d0d5dd' } }] }]
      });
    }

    watch(activeTab, value => {
      if (value === 'report') nextTick(renderCharts);
    });
    watch(defectList, () => { if (activeTab.value === 'report') nextTick(renderCharts); });
    function resizeCharts() {
      if (levelChart) levelChart.resize();
      if (typeChart) typeChart.resize();
    }

    onMounted(() => {
      initialize();
      window.addEventListener('resize', resizeCharts);
    });
    onBeforeUnmount(() => {
      if (socket) socket.close();
      [socketReconnect, heartbeatTimer, fallbackPollTimer, defectPollTimer, toastTimer].forEach(timer => timer && clearTimeout(timer));
      if (levelChart) levelChart.dispose();
      if (typeChart) typeChart.dispose();
      window.removeEventListener('resize', resizeCharts);
    });

    return {
      activeTab, options, selectedMode, selectedDestination, selectedCamera, cameras, defectList, toast,
      backendOnline, rosOnline, odom, navigation, safety, stateText, stateDotClass, backendTitle, rosTitle,
      taskLocked, canStart, progressWidth, onlineCameraCount, cameraAvailable, cameraStreamUrl, cameraBBox,
      selectCamera, reportStats, startNavigation, chooseJunction, sendCommand,
      formatNumber, formatMetric, formatTime, positionText, levelClass, cameraSnapshotUrl, generateReport
    };
  }
}).mount('#app');
