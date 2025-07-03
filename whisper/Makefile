run:
	@PORCUPINE_ACCESS_KEY="hIhjs+R+zvJewDx9W8p+olnqxfe5lFC//fihuAQSJQdI7uEIpteXFQ==" \
	OLLAMA_API_URL="http://localhost:11434" \
	.venv/bin/python speech_recog.py

kill:
	ps aux | grep speech_recog.py | grep -v grep | awk '{print $$2}' | xargs kill
	killall coreaudiod || true