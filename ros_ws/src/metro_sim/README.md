# metro_sim

Gazebo 仿真资源。当前先集成新隧道，小车模型后续再加入。

## 预览新隧道

```bash
./scripts/open_subway_tunnel.sh
```

目录说明：

- `models/subway_tunnel/`：带贴图的 DAE、简化碰撞网格和 Blender 源文件
- `worlds/subway_tunnel.world`：只包含隧道与预览灯光，不包含小车
- `scripts/open_subway_tunnel.sh`：设置 `GAZEBO_MODEL_PATH` 并启动 Gazebo Classic
