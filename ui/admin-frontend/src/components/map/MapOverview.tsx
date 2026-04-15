import { useEffect, useRef } from 'react';
import { useSystemStore } from '@/stores/useSystemStore';
import { getStatusColor } from '@/lib/utils';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';

export function MapOverview() {
  const mapRef = useRef<HTMLDivElement>(null);
  const mapInstanceRef = useRef<L.Map | null>(null);
  const markersRef = useRef<L.CircleMarker[]>([]);
  const devices = useSystemStore((s) => s.devices);

  useEffect(() => {
    if (!mapRef.current || mapInstanceRef.current) return;

    const map = L.map(mapRef.current, {
      center: [22.5435, 108.3745],
      zoom: 16,
      zoomControl: false,
      attributionControl: false,
    });

    L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
      maxZoom: 19,
    }).addTo(map);

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

    devices.forEach((device) => {
      const color = getStatusColor(device.status);
      const marker = L.circleMarker(device.position, {
        radius: 8,
        fillColor: color,
        color: color,
        weight: 2,
        opacity: 0.8,
        fillOpacity: 0.5,
      })
        .addTo(map)
        .bindPopup(
          `<div style="font-size:12px;color:#333;">
            <b>${device.name}</b><br/>
            状态: ${device.status}<br/>
            电量: ${device.battery}%
          </div>`
        );

      const pulseMarker = L.circleMarker(device.position, {
        radius: 16,
        fillColor: color,
        color: 'transparent',
        weight: 0,
        fillOpacity: 0.15,
      }).addTo(map);

      markersRef.current.push(marker, pulseMarker);
    });
  }, [devices]);

  return <div ref={mapRef} className="w-full h-full rounded-lg overflow-hidden" />;
}
