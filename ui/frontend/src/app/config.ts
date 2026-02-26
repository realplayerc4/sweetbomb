
const isDev = import.meta.env?.DEV;
const FALLBACK_HOST = 'http://localhost:8000';
const HOST = isDev ? FALLBACK_HOST : window.location.origin;

export const API_BASE = `${HOST}/api`;
export const SOCKET_URL = HOST;
