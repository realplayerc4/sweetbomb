import { useEffect, useRef } from 'react';
import { useSystemStore } from '@/stores/useSystemStore';
import { useSystemModeStore } from '@/stores/useSystemModeStore';
import { getStatusColor } from '@/lib/utils';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';

export function MapMonitor() {
  const mapRef = useRef<HTMLDivElement>(null);
  const mapInstanceRef = useRef<L.Map | null>(null);
  const markersRef = useRef<L.CircleMarker[]>([]);
  const devices = useSystemStore((s) => s.devices);
  const monitorMode = useSystemModeStore((s) => s.monitorMode);
  const selectedRobot = useSystemModeStore((s) => s.selectedRobot);

  useEffect(() => {
    if (!mapRef.current || mapInstanceRef.current) return;

    const map = L.map(mapRef.current, {
      center: [22.5435, 108.3745],
      zoom: 17,
      zoomControl: false,
      attributionControl: false,
    });

    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
      maxZoom: 19,
    }).addTo(map);

    L.control.zoom({ position: 'bottomright' }).addTo(map);

    const zoneStyle = {
      color: '#00d4ff',
      weight: 1,
      fillColor: '#00d4ff',
      fillOpacity: 0.05,
      dashArray: '5 5',
    };

    L.rectangle(
      [[22.5425, 108.3735], [22.5445, 108.3760]],
      { ...zoneStyle, color: '#1677ff' }
    ).addTo(map).bindPopup('<b style="color:#333">A区 - 装卸作业区</b>');

    L.rectangle(
      [[22.5445, 108.3735], [22.5455, 108.3760]],
      { ...zoneStyle, color: '#52c41a' }
    ).addTo(map).bindPopup('<b style="color:#333">B区 - 糖堆存放区</b>');

    L.rectangle(
      [[22.5415, 108.3735], [22.5425, 108.3760]],
      { ...zoneStyle, color: '#fa8c16' }
    ).addTo(map).bindPopup('<b style="color:#333">C区 - 充电维护区</b>');

    mapInstanceRef.current = map;

    return () => {
      map.remove();
      mapInstanceRef.current = null;
    };
  }, []);

  useEffect(() => {
    const map = mapInstanceRef.current;
    if (!map) return;

    markersRef.current.forEach((m) => m.remove());
    markersRef.current = [];

    const filteredDevices = monitorMode === 'single'
      ? devices.filter((d) => d.device_id === selectedRobot)
      : devices;

    filteredDevices.forEach((device) => {
      const color = getStatusColor(device.status);
      const isSelected = device.device_id === selectedRobot;

      const marker = L.circleMarker(device.position, {
        radius: isSelected ? 14 : 10,
        fillColor: color,
        color: isSelected ? '#00d4ff' : '#ffffff',
        weight: isSelected ? 3 : 2,
        opacity: 0.9,
        fillOpacity: 0.7,
      })
        .addTo(map)
        .bindPopup(
          `<div style="font-size:13px;color:#333;min-width:130px;">
            <b>${device.name}</b><br/>
            状态: <span style="color:${color}">${device.status}</span><br/>
            电量: ${device.battery}%
            ${device.task_id ? `<br/>任务: ${device.task_id}` : ''}
          </div>`
        );

      const pulseMarker = L.circleMarker(device.position, {
        radius: isSelected ? 25 : 20,
        fillColor: color,
        color: 'transparent',
        weight: 0,
        fillOpacity: isSelected ? 0.2 : 0.12,
      }).addTo(map);

      L.marker(device.position, {
        icon: L.divIcon({
          className: '',
          html: `<div style="
            width:80px;
            text-align:center;
            font-size:10px;
            color:#fff;
            background:rgba(10,20,40,0.85);
            padding:2px 6px;
            border-radius:3px;
            border:1px solid ${isSelected ? '#00d4ff' : color}60;
            white-space:nowrap;
            font-weight:bold;
            transform:translate(-50%,-150%);
            ${isSelected ? 'box-shadow:0 0 8px rgba(0,212,255,0.4);' : ''}
          ">${device.name}</div>`,
          iconSize: [0, 0],
        }),
      }).addTo(map);

      markersRef.current.push(marker, pulseMarker);
    });
  }, [devices, monitorMode, selectedRobot]);

  return <div ref={mapRef} className="w-full h-full" />;
}
