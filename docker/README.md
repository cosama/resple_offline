# Docker

The build context must be the RESPLE framework root because the image needs
both `core/` and the pinned source under `upstream/`.

From the `slam_benchmark` repository root:

```bash
docker build \
  -f frameworks/resple/docker/Dockerfile \
  -t resple-offline \
  frameworks/resple
```

Smoke test:

```bash
docker run --rm resple-offline
```
