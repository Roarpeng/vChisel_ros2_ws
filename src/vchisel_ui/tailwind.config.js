/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {
      colors: {
        // Bosch品牌色
        'bosch-red': '#E20015',
        'bosch-gray': {
          100: '#F5F5F5',
          200: '#E0E0E0',
          300: '#BDBDBD',
          400: '#9E9E9E',
          500: '#757575',
          600: '#616161',
          700: '#424242',
          800: '#303030',
          900: '#212121',
        },
        // 状态色
        'status-ok': '#4CAF50',
        'status-warning': '#FF9800',
        'status-error': '#F44336',
        'status-info': '#2196F3',
      },
    },
  },
  plugins: [],
}
