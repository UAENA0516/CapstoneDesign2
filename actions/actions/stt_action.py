import pyaudio

if __name__ == '__main__':
    audio = pyaudio.PyAudio()

    for index in range(audio.get_device_count()):
        desc = audio.get_device_info_by_index(index)
        print("DEVICE: {device}, INDEX: {index}, RATE: {rate} ".format(
            device=desc["name"], index=index, rate=int(desc["defaultSampleRate"])))


    import speech_recognition as sr

    r = sr.Recognizer()
    with sr.Microphone(device_index=5) as source:
        print('동작중')
        audio = r.listen(source)
    text = r.recognize_google(audio, language='ko')
    try:
        text = r.recognize_google(audio, language='ko')
        print(text)
    except sr.UnknownValueError:
        print("동작 실패")
    except sr.RequestError as e:
        print('요청 실패 : {0}'.format(e))