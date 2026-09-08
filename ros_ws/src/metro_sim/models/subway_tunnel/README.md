# Subway tunnel model

This directory contains the Gazebo Classic 11 tunnel environment model.

- `meshes/tunnel.dae`: visual mesh with exported colors and the baked concrete texture.
- `meshes/concrete_diffuse.png`: baked diffuse texture used by the DAE material.
- `meshes/tunnel_collision.stl`: simplified collision mesh.
- `source/tunnel.blend`: Blender 5.2 source file for future visual edits.

The visual and collision meshes are intentionally separate. When re-exporting from Blender,
exclude objects whose names contain `Collision` from the visual mesh.
