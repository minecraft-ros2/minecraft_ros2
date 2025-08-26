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
                { text: 'ROS 2 Tutorial', link: `/tutorial/index`}
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
                    { text: 'Debug Messages', link: './doc_debug_msgs' },
                    { text: 'Command Execution', link: './doc_cmd' },
                  ]
                }
            ],
            "/tutorial/": [
                {
                  text: 'Tutorial',
                  items: [
                    { text: 'Index', link: '/tutorial/index'},
                    { text: 'topic', link: '/tutorial/00_topic'},
                    { text: 'rviz2', link: '/tutorial/01_rviz2'},
                    { text: 'Control Player', link: '/tutorial/02_control_player'},
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
                { text: 'ROS 2チュートリアル', link: `jp/tutorial/index`}
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
                    { text: 'デバッグメッセージ', link: './doc_debug_msgs' },
                    { text: 'コマンド実行', link: './doc_cmd' },
                  ]
                }
              ],
              "/jp/tutorial/": [
                {
                  text: 'チュートリアル',
                  items: [
                    { text: '目次', link: '/jp/tutorial/index'},
                    { text: 'トピック', link: '/jp/tutorial/00_topic'},
                    { text: 'RViz2', link: '/jp/tutorial/01_rviz2'},
                    { text: 'プレイヤー操作', link: '/jp/tutorial/02_control_player'},
                    { text: '自己位置推定', link: '/jp/tutorial/05_localization'},
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