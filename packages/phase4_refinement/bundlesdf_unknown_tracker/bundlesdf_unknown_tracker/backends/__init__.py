"""BundleSDF backend implementations.

Keep heavy imports (torch, open3d, pytorch3d, BundleSDF's C++/CUDA ops)
inside individual backend modules so this package is import-safe on
CPU-only hosts and in lint/test environments.
"""
