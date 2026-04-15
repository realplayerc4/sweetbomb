import { ConfigProvider, theme } from 'antd';
import { DashboardLayout } from './components/layout/DashboardLayout';

export default function App() {
  return (
    <ConfigProvider
      theme={{
        algorithm: theme.darkAlgorithm,
        token: {
          colorPrimary: '#00d4ff',
          colorBgContainer: '#0f172a',
          colorBgElevated: '#0f172a',
          colorBorder: 'rgba(0,212,255,0.2)',
          colorText: '#cbd5e1',
          colorTextSecondary: '#64748b',
          borderRadius: 6,
          fontFamily: "'Source Han Sans', 'Noto Sans SC', 'Roboto', -apple-system, sans-serif",
          fontSize: 16,
        },
      }}
    >
      <DashboardLayout />
    </ConfigProvider>
  );
}
