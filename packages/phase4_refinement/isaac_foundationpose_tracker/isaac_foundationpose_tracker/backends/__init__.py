"""FoundationPose backend implementations.

Keep heavy imports (torch, kaolin, nvdiffrast, foundationpose) inside
individual backend modules so this package is import-safe on CPU-only
hosts and in lint/test environments.
"""
