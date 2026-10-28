FROM python:3.12-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    libosmesa6 libgl1 libglib2.0-0 nodejs npm git ca-certificates \
    && rm -rf /var/lib/apt/lists/*
RUN npm install --global @openai/codex@0.165.0
WORKDIR /opt/robofrontier
COPY pyproject.toml README.md ./
COPY robofrontier ./robofrontier
RUN pip install --no-cache-dir .
WORKDIR /workspace
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa LIBGL_ALWAYS_SOFTWARE=true GALLIUM_DRIVER=llvmpipe CUDA_VISIBLE_DEVICES=""
ENTRYPOINT ["python", "-m", "scripts.luna_cpu"]
