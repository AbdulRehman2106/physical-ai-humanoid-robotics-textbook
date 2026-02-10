export interface Theme {
  id: string;
  name: string;
  icon: string;
  colors: {
    primary: string;
    primaryDark: string;
    primaryDarker: string;
    primaryDarkest: string;
    primaryLight: string;
    primaryLighter: string;
    primaryLightest: string;
  };
}

export const themes: Theme[] = [
  {
    id: 'purple',
    name: 'Default Purple',
    icon: '💜',
    colors: {
      primary: '#667eea',
      primaryDark: '#5a67d8',
      primaryDarker: '#4c51bf',
      primaryDarkest: '#434190',
      primaryLight: '#7c3aed',
      primaryLighter: '#8b5cf6',
      primaryLightest: '#a78bfa',
    },
  },
  {
    id: 'ocean',
    name: 'Ocean Blue',
    icon: '🌊',
    colors: {
      primary: '#0ea5e9',
      primaryDark: '#0284c7',
      primaryDarker: '#0369a1',
      primaryDarkest: '#075985',
      primaryLight: '#38bdf8',
      primaryLighter: '#7dd3fc',
      primaryLightest: '#bae6fd',
    },
  },
  {
    id: 'sunset',
    name: 'Sunset Orange',
    icon: '🌅',
    colors: {
      primary: '#f97316',
      primaryDark: '#ea580c',
      primaryDarker: '#c2410c',
      primaryDarkest: '#9a3412',
      primaryLight: '#fb923c',
      primaryLighter: '#fdba74',
      primaryLightest: '#fed7aa',
    },
  },
  {
    id: 'forest',
    name: 'Forest Green',
    icon: '🌲',
    colors: {
      primary: '#10b981',
      primaryDark: '#059669',
      primaryDarker: '#047857',
      primaryDarkest: '#065f46',
      primaryLight: '#34d399',
      primaryLighter: '#6ee7b7',
      primaryLightest: '#a7f3d0',
    },
  },
  {
    id: 'midnight',
    name: 'Midnight Blue',
    icon: '🌙',
    colors: {
      primary: '#3b82f6',
      primaryDark: '#2563eb',
      primaryDarker: '#1d4ed8',
      primaryDarkest: '#1e40af',
      primaryLight: '#60a5fa',
      primaryLighter: '#93c5fd',
      primaryLightest: '#dbeafe',
    },
  },
  {
    id: 'rose',
    name: 'Rose Pink',
    icon: '🌹',
    colors: {
      primary: '#ec4899',
      primaryDark: '#db2777',
      primaryDarker: '#be185d',
      primaryDarkest: '#9f1239',
      primaryLight: '#f472b6',
      primaryLighter: '#f9a8d4',
      primaryLightest: '#fce7f3',
    },
  },
  {
    id: 'cyber',
    name: 'Cyber Neon',
    icon: '⚡',
    colors: {
      primary: '#06b6d4',
      primaryDark: '#0891b2',
      primaryDarker: '#0e7490',
      primaryDarkest: '#155e75',
      primaryLight: '#22d3ee',
      primaryLighter: '#67e8f9',
      primaryLightest: '#cffafe',
    },
  },
  {
    id: 'autumn',
    name: 'Autumn Gold',
    icon: '🍂',
    colors: {
      primary: '#f59e0b',
      primaryDark: '#d97706',
      primaryDarker: '#b45309',
      primaryDarkest: '#92400e',
      primaryLight: '#fbbf24',
      primaryLighter: '#fcd34d',
      primaryLightest: '#fde68a',
    },
  },
];

export const applyTheme = (theme: Theme) => {
  const root = document.documentElement;
  root.style.setProperty('--ifm-color-primary', theme.colors.primary);
  root.style.setProperty('--ifm-color-primary-dark', theme.colors.primaryDark);
  root.style.setProperty('--ifm-color-primary-darker', theme.colors.primaryDarker);
  root.style.setProperty('--ifm-color-primary-darkest', theme.colors.primaryDarkest);
  root.style.setProperty('--ifm-color-primary-light', theme.colors.primaryLight);
  root.style.setProperty('--ifm-color-primary-lighter', theme.colors.primaryLighter);
  root.style.setProperty('--ifm-color-primary-lightest', theme.colors.primaryLightest);
};

export const getThemeById = (id: string): Theme => {
  return themes.find(theme => theme.id === id) || themes[0];
};
