import { motion } from 'framer-motion';

interface RobotDisplayProps {
  power: number;
  armRotation: number;
  headRotation: number;
}

export function RobotDisplay({ power, armRotation, headRotation }: RobotDisplayProps) {
  const isActive = power > 0;

  return (
    <div className="relative w-full h-full flex items-center justify-center">
      <svg
        width="300"
        height="400"
        viewBox="0 0 300 400"
        fill="none"
        xmlns="http://www.w3.org/2000/svg"
        className="drop-shadow-2xl"
      >
        {/* Body */}
        <motion.rect
          x="100"
          y="150"
          width="100"
          height="120"
          rx="10"
          fill={isActive ? "#3b82f6" : "#6b7280"}
          stroke="#1e293b"
          strokeWidth="3"
          animate={{
            fill: isActive ? "#3b82f6" : "#6b7280",
          }}
          transition={{ duration: 0.3 }}
        />

        {/* Chest Panel */}
        <rect x="120" y="170" width="60" height="80" rx="5" fill="#1e293b" opacity="0.3" />
        <circle cx="150" cy="200" r="8" fill={isActive ? "#10b981" : "#374151"}>
          {isActive && (
            <animate
              attributeName="opacity"
              values="1;0.3;1"
              dur="2s"
              repeatCount="indefinite"
            />
          )}
        </circle>

        {/* Energy Lines */}
        {isActive && (
          <>
            <motion.line
              x1="135"
              y1="220"
              x2="165"
              y2="220"
              stroke="#10b981"
              strokeWidth="2"
              initial={{ pathLength: 0 }}
              animate={{ pathLength: 1 }}
              transition={{ duration: 1, repeat: Infinity }}
            />
            <motion.line
              x1="135"
              y1="230"
              x2="165"
              y2="230"
              stroke="#10b981"
              strokeWidth="2"
              initial={{ pathLength: 0 }}
              animate={{ pathLength: 1 }}
              transition={{ duration: 1, repeat: Infinity, delay: 0.2 }}
            />
          </>
        )}

        {/* Head */}
        <motion.g
          animate={{
            rotate: headRotation,
          }}
          transition={{ type: "spring", stiffness: 100 }}
          style={{ originX: "150px", originY: "100px" }}
        >
          <rect
            x="125"
            y="80"
            width="50"
            height="50"
            rx="8"
            fill={isActive ? "#60a5fa" : "#6b7280"}
            stroke="#1e293b"
            strokeWidth="3"
          />

          {/* Eyes */}
          <circle cx="140" cy="100" r="6" fill={isActive ? "#fbbf24" : "#374151"}>
            {isActive && (
              <animate
                attributeName="opacity"
                values="1;0.2;1"
                dur="3s"
                repeatCount="indefinite"
              />
            )}
          </circle>
          <circle cx="160" cy="100" r="6" fill={isActive ? "#fbbf24" : "#374151"}>
            {isActive && (
              <animate
                attributeName="opacity"
                values="1;0.2;1"
                dur="3s"
                repeatCount="indefinite"
              />
            )}
          </circle>

          {/* Antenna */}
          <line x1="150" y1="80" x2="150" y2="60" stroke="#1e293b" strokeWidth="3" />
          <circle cx="150" cy="60" r="5" fill={isActive ? "#ef4444" : "#6b7280"}>
            {isActive && (
              <animate
                attributeName="fill"
                values="#ef4444;#fbbf24;#ef4444"
                dur="1s"
                repeatCount="indefinite"
              />
            )}
          </circle>

          {/* Mouth */}
          <line x1="135" y1="115" x2="165" y2="115" stroke="#1e293b" strokeWidth="2" />
        </motion.g>

        {/* Left Arm */}
        <motion.g
          animate={{
            rotate: -armRotation,
          }}
          transition={{ type: "spring", stiffness: 100 }}
          style={{ originX: "90px", originY: "165px" }}
        >
          <rect
            x="70"
            y="160"
            width="30"
            height="70"
            rx="8"
            fill={isActive ? "#3b82f6" : "#6b7280"}
            stroke="#1e293b"
            strokeWidth="3"
          />
          <circle cx="85" cy="235" r="8" fill="#1e293b" />
        </motion.g>

        {/* Right Arm */}
        <motion.g
          animate={{
            rotate: armRotation,
          }}
          transition={{ type: "spring", stiffness: 100 }}
          style={{ originX: "210px", originY: "165px" }}
        >
          <rect
            x="200"
            y="160"
            width="30"
            height="70"
            rx="8"
            fill={isActive ? "#3b82f6" : "#6b7280"}
            stroke="#1e293b"
            strokeWidth="3"
          />
          <circle cx="215" cy="235" r="8" fill="#1e293b" />
        </motion.g>

        {/* Legs */}
        <rect
          x="110"
          y="270"
          width="35"
          height="80"
          rx="8"
          fill={isActive ? "#3b82f6" : "#6b7280"}
          stroke="#1e293b"
          strokeWidth="3"
        />
        <rect
          x="155"
          y="270"
          width="35"
          height="80"
          rx="8"
          fill={isActive ? "#3b82f6" : "#6b7280"}
          stroke="#1e293b"
          strokeWidth="3"
        />

        {/* Feet */}
        <rect x="100" y="350" width="50" height="20" rx="5" fill="#1e293b" />
        <rect x="150" y="350" width="50" height="20" rx="5" fill="#1e293b" />
      </svg>

      {/* Power Indicator */}
      <div className="absolute top-4 left-4">
        <div className="flex items-center gap-2 bg-slate-900/80 backdrop-blur-sm px-3 py-2 rounded-lg border border-slate-700">
          <div className={`w-2 h-2 rounded-full ${isActive ? 'bg-green-500 animate-pulse' : 'bg-gray-500'}`} />
          <span className="text-xs text-white">{isActive ? 'ONLINE' : 'OFFLINE'}</span>
        </div>
      </div>
    </div>
  );
}
