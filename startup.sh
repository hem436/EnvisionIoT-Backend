flask db upgrade
gunicorn --worker-class eventlet -w 1 --threads 4 --timeout 60 --access-logfile \
    '-' --error-logfile '-' --bind=0.0.0.0:8000 \
     app:app
