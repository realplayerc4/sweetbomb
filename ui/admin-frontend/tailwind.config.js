/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        'bg-primary': 'var(--color-bg-primary)',
        'bg-secondary': 'var(--color-bg-secondary)',
        'bg-tertiary': 'var(--color-bg-tertiary)',
        'tech-blue': 'var(--color-tech-blue)',
        'tech-cyan': 'var(--color-tech-cyan)',
        'tech-green': 'var(--color-tech-green)',
        'tech-orange': 'var(--color-tech-orange)',
        'tech-red': 'var(--color-tech-red)',
        'text-primary': 'var(--color-text-primary)',
        'text-secondary': 'var(--color-text-secondary)',
        'text-muted': 'var(--color-text-muted)',
      },
      fontFamily: {
        sans: ['Source Han Sans', 'Noto Sans SC', 'Roboto', 'sans-serif'],
      },
    },
  },
  plugins: [],
}
