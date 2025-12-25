import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Learn to build intelligent robots with ROS 2, Gazebo, NVIDIA Isaac, and LLMs',
  favicon: 'img/favicon.svg',

  url: 'https://TALHAFAROOQ136.github.io',
  baseUrl: '/physical_ai/',

  organizationName: 'TALHAFAROOQ136',
  projectName: 'physical_ai',
  trailingSlash: false,

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Custom fields accessible via useDocusaurusContext() and window.__DOCUSAURUS__
  customFields: {
    apiUrl: 'http://localhost:8000/api/v1', // Change to production URL for deployment
  },

  // Performance optimizations
  // Note: Enable experimental_faster after installing @docusaurus/faster
  // future: {
  //   experimental_faster: true,
  // },
  headTags: [
    // Preconnect to external resources
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossorigin: 'anonymous',
      },
    },
    // DNS prefetch for API
    {
      tagName: 'link',
      attributes: {
        rel: 'dns-prefetch',
        href: 'https://api.physical-ai-textbook.com',
      },
    },
  ],

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
      },
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/TALHAFAROOQ136/physical_ai/tree/main/frontend/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
          // Performance: async sidebar
          async sidebarItemsGenerator({ defaultSidebarItemsGenerator, ...args }) {
            const sidebarItems = await defaultSidebarItemsGenerator(args);
            return sidebarItems;
          },
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/social-card.svg',
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
          label: 'Course',
        },
        // Dashboard disabled - requires backend
        // {
        //   href: '/dashboard',
        //   label: 'Dashboard',
        //   position: 'left',
        // },
        {
          type: 'localeDropdown',
          position: 'right',
        },
        {
          href: 'https://github.com/TALHAFAROOQ136/physical_ai',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Course',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-1-ros2',
            },
            {
              label: 'Module 2: Simulation',
              to: '/docs/module-2-simulation',
            },
          ],
        },
        {
          title: 'Advanced',
          items: [
            {
              label: 'Module 3: NVIDIA Isaac',
              to: '/docs/module-3-isaac',
            },
            {
              label: 'Module 4: VLA',
              to: '/docs/module-4-vla',
            },
            {
              label: 'Capstone Project',
              to: '/docs/capstone',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/TALHAFAROOQ136/physical_ai',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'json', 'cpp', 'cmake'],
    },
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
