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
        },
    },
    plugins: [],
}

