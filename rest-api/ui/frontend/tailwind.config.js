/** @type {import('tailwindcss').Config} */
export default {
    content: [
        "./index.html",
        "./src/**/*.{js,ts,jsx,tsx}",
    ],
    // 确保动态使用的颜色类名被编译
    safelist: [
        'text-red-500', 'text-yellow-500', 'text-green-500',
        'bg-red-500', 'bg-yellow-500', 'bg-green-500',
    ],
    theme: {
        extend: {
            fontFamily: {
                mono: ['JetBrains Mono', 'monospace'],
            },
            animation: {
                'pulse-glow': 'pulseGlow 2s cubic-bezier(0.4, 0, 0.6, 1) infinite',
            },
            keyframes: {
                pulseGlow: {
                    '0%, 100%': { boxShadow: '0 0 15px rgba(253, 128, 46, 0.4)', borderColor: 'rgba(253, 128, 46, 0.8)' },
                    '50%': { boxShadow: '0 0 5px rgba(253, 128, 46, 0.1)', borderColor: 'rgba(253, 128, 46, 0.3)' },
                }
            }
        },
    },
    plugins: [],
}

