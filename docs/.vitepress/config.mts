import { defineConfig } from 'vitepress'

export default defineConfig({
  base: "/minecraft_ros2/",
  title: "Minecraft ROS 2",
  description: "Documentation of the minecraft_ros2",
  locales: {
    root: {
        label: `English`,
        lang: `en`,
        themeConfig: {
            nav: [
                { text: 'Home', link: `/`},
                { text: 'Document', link: `/en/documentation/install_docker`},
                { text: 'ROS2 Training', link: `en/index`}
            ],
            sidebar: {
              "/en/documentation/": [{
                text: 'Installation',
                items: [
                  { text: 'Setup with Docker', link: '/en/documentation/install_docker' },
                  { text: 'Source Install', link: '/en/documentation/install_source' },
                ]
              }]
            }
        }
    },
    jp: {
        label: `日本語`,
        lang: `jp`,
        link: `/jp/index`,
        themeConfig: {
            nav: [
                { text: 'ホーム', link: `jp/index`},
                { text: 'ドキュメント', link: `/jp/documentation/install_docker`},
                { text: 'ROS2トレーニング', link: `jp/index`}
            ],
            sidebar: {
              "/jp/documentation/": [{
                text: 'インストール',
                items: [
                  { text: 'Dockerセットアップ', link: '/jp/documentation/install_docker' },
                  { text: 'ソースインストール', link: '/jp/documentation/install_source' },
                ]
              }]
            }
        },
    },
  },
  themeConfig: {
    socialLinks: [
      { icon: 'github', link: 'https://github.com/minecraft-ros2/minecraft_ros2' }
    ],
  }
})