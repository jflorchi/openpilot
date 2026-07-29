FROM python:3.11-slim

WORKDIR /app

# ffmpeg stitches a route's qcamera segments into one playable mp4
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg \
 && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt gunicorn

COPY server.py store.py web.py test_server.py test_web.py ./
COPY templates/ ./templates/
COPY static/ ./static/

# Uploads are large and slow over LTE, so give them a generous timeout and use
# threaded workers -- the upload handler is IO bound, streaming to disk.
ENV DATA_DIR=/data PORT=8080
EXPOSE 8080
VOLUME ["/data"]

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s \
  CMD python -c "import urllib.request;urllib.request.urlopen('http://127.0.0.1:8080/health')"

CMD ["gunicorn", "-b", "0.0.0.0:8080", "-k", "gthread", "-w", "2", "--threads", "8", \
     "--timeout", "600", "--access-logfile", "-", "server:app"]
