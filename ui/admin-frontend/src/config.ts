const isDev = import.meta.env?.DEV;
const API_PORT = '8000';
const HOST = isDev
  ? `${window.location.protocol}//${window.location.hostname}:${API_PORT}`
  : window.location.origin;

export const API_BASE = `${HOST}/api`;
export const SOCKET_URL = HOST;

export const REFRESH_INTERVAL = 2000;
export const HEARTBEAT_INTERVAL = 30000;
export const MAX_RECONNECT_ATTEMPTS = 5;
