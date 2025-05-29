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
                { text: 'Document', link: `/documentation/doc_sensors`},
                // { text: 'ROS2 Training', link: `/`}
            ],
            sidebar: {
              "/documentation/": [
                {
                  text: 'Introduction',
                  items: [
                    { text: 'What is Minecraft ROS 2 ?', link: '/documentation/what_is' },
                    { text: 'Setup with Docker', link: '/documentation/install_docker' },
                    { text: 'Source Install', link: '/documentation/install_source' },
                  ]
                },
                {
                  text: 'Reference',
                  items: [
                    { text: 'Sensors', link: './doc_sensors' },
                    { text: 'Player Control', link: './doc_player' },
                  ]
                }
            ]
            }
        }
    },
    jp: {
        label: `日本語`,
        lang: `jp`,
        link: `/jp/`,
        themeConfig: {
            nav: [
                { text: 'ホーム', link: `jp/index`},
                { text: 'ドキュメント', link: `/jp/documentation/doc_sensors`},
                // { text: 'ROS2トレーニング', link: `jp/index`}
            ],
            sidebar: {
              "/jp/documentation/": [
                {
                  text: '紹介',
                  items: [
                    { text: 'はじめに', link: './what_is' },
                    { text: 'Dockerセットアップ', link: './install_docker' },
                    { text: 'ソースインストール', link: './install_source' },
                  ]
                },
                {
                  text: 'リファレンス',
                  items: [
                    { text: 'センサー', link: './doc_sensors' },
                    { text: 'プレイヤー操作', link: './doc_player' },
                  ]
                }
              ]
            }
        },
    },
  },
  themeConfig: {
    socialLinks: [
      { icon: 'github', link: 'https://github.com/minecraft-ros2/minecraft_ros2' }
    ],
    search: {
      provider: 'local'
    }
  }
})