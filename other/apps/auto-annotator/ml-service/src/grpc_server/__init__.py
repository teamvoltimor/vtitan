"""gRPC compute workers exposing SAM segmentation, augmentation, and training.

These servers wrap the surviving Python ML compute behind the language-neutral
``autoannotator.v1`` contract so the Go orchestration API can call them. The Go
API owns the SQLite manifest DB; these workers only touch the filesystem.
"""
