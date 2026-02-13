// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const {themes: prismThemes} = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Embodied Intelligence and Robotics Systems',
  favicon: 'img/favicon.svg',

  url: 'https://physical-ai-textbook.com',
  baseUrl: '/',

  organizationName: 'physical-ai',
  projectName: 'physical-ai-textbook',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: undefined,
          remarkPlugins: [],
          rehypePlugins: [],
          exclude: [
            '**/specification.md',
            '**/US*-validation.md',
            '**/_template/**',
          ],
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  scripts: [
    {
      src: '/config.js',
      async: false,
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.jpg',
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Chapters',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learn',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Physical AI Intro',
                to: '/docs/chapters/physical-ai-intro',
              },
              {
                label: 'ROS 2 Fundamentals',
                to: '/docs/chapters/ros2-fundamentals',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'NVIDIA Isaac Sim',
                href: 'https://developer.nvidia.com/isaac-sim',
              },
            ],
          },
          {
            title: 'Connect',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/AbdulRehman2106',
              },
              {
                label: 'LinkedIn',
                href: 'https://www.linkedin.com/in/abdul-rehman-2213012b9/',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built with Docusaurus. Created by Abdul Rehman.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'yaml', 'markup'],
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
    }),
};

module.exports = config;
