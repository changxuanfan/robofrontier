# 配置入口

| 路径 | 含义 |
| --- | --- |
| `task_families/current.json` | 四类任务、GPT/Oracle用途、公开材料及源剪辑的统一选择 |
| `task_families/<family>/public/` | 可交给新策略的已审查静态材料 |
| `harnesses/policy.json` | 新GPT准备的默认共享控制与协作配置 |
| `backgrounds/robo_factory.json` | 当前V7工厂外观、布局与资源复用 |
| `suites/frontier-current-oracle.json` | 36桌冻结Oracle源的工厂布局输入 |
| `environments/` | 不可就地改写的场景修订和物理元数据 |

其余家族registry、catalog、harness及旧任务配置是原实验的版本依据。旧
`suites/frontier-v2.json`和通用生成器保留兼容，不代替current.json的用途选择。
`source-compatibility.json`只证明共享改动对历史Mosaic/Dial检查的明确范围，
不修改原指纹，不证明Relay旧评分与新评分等价。
