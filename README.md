
# Camera project

Идея проекта заключалась в том, чтобы создать камеру, реагирующую на движение. В итоговом варианте камера реагирует на синий цвет, а также может двигаться с помощью пульта. Мозгом нашего проекта стала [Raspberry pi 4](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) с камерой [Camera Module 3.](https://www.raspberrypi.com/products/camera-module-3/)  Чтобы камера была подвижной по осям Ox и Oy, мы использовали два [сервопривода](https://docs.sunfounder.com/projects/ultimate-sensor-kit/en/latest/components_basic/27-component_servo.html). Для включения/выключения камеры и её управления был задействован [ИК-приемник,](https://roboshop.spb.ru/sensors/infrakrasnye-datchiki/tl1838), управляемый [Arduino Nano](https://3d-diy.ru/blog/arduino-nano/?srsltid=AfmBOoobsfLvRHXmztk4oDekqijM6OquVmeA1C7HS3Jm3zs6FXj-3EUY).

![project_image](images/project_photo.jpg)

![project_gif](./images/project_usage.gif)

# Настройка Raspberry pi

## Установка OS и VNC

Накатили Raspberry pi OS (все необходимое можно найти
[здесь](https://www.raspberrypi.com/software/)).
Советую сразу настроить удаленный доступ к графическому интерфейсу
[VNC для удобной работы с малинкой.](https://habr.com/ru/sandbox/148360/)

## Настройка виртуального окружения для работы с python

Для нормальной работы с библиотекой picamra2 лучше установить виртуальное окружение:

```sh
sudo apt install python3-venv
python3 -m venv —system-site-packages env
```

Для активации:

```sh
source env/bin/activate
```

Соответственно, чтобы выйти из окружения: `deactivate`.  
Для работы с пинами малинки использовали библиотеку [pigpio](https://abyz.me.uk/rpi/pigpio/pigpiod.html), для работы с которой нужно запустить pigpio library as a daemon:

```sh
sudo pigpiod
```

## Настройка ИК-приемника

## ИК-приемник на Raspberry pi (LIRC)

Вообще говоря, сначала мы пытались обрабатывать кнопки всё так же на малинке через библиотеку LIRC. Нужно было немного изменить конфиг малинки, добавив несколько строчек в него ([вот здесь всё детально расписано](https://www.instructables.com/Setup-IR-Remote-Control-Using-LIRC-for-the-Raspber/)).

Схему подключения ИК-приёмника к малинке не буду вставлять — тут всё просто: GND к GND, VCC к 3.3V, OUT к, например, 18 порту малинки (GPIO18 имеет альтернативную функцию **PCM_CLK**, которая позволяет использовать аппаратный PWM, что критично для стабильного декодирования ИК-сигналов).

Проверяем, что ИК-приёмник работает:

```sh
mode2 -d /dev/lirc0
```

Если на экран выводится **pulse/space codes**, то всё круто — работает.
Второй шаг — найти конфиг для нашего пульта. У нас это обычный китайский пульт Car MP3.

![remote_image](images/remote_photo.png)

Но внимание! В общем [архиве](https://lirc.sourceforge.net/remotes/) не нашлось конфига! Однако мы не отчаялись. В таком случае можно вручную записать свой конфиг, используя утилиту irrecord. Как и было написано в указанной ранее [статье](https://www.instructables.com/Easy-Setup-IR-Remote-Control-Using-LIRC-for-the-Ra/): "I was VERY unsuccessful trying to create a file using this utility despite much effort".
Разочаровавшись, мы пошли вглубь интернета с целью всё-таки найти конфиг под наш пульт. Если кому-то понадобится, ссылка есть на [этом сайте](https://elchupanibrei.livejournal.com/43594.html) рядом с изображением пульта.
Добавляем наш конфиг в `/etc/lirc/lircd.conf.d/` и включаем его в `/etc/lirc/lircd.conf`:

```sh
mv carMP3.conf /etc/lirc/lircd.conf.d/
sudo echo 'include "lircd.conf.d/*.conf"' >> /etc/lirc/lircd.conf 
```

Перезапускаем LIRC и проверяем работает ли конфиг для пульта:

```sh
sudo service lirc start
irw /var/run/lirc/lircd
```

После нажатия на кнопку пульта должно выводиться что-то типа:

```sh
0000000000ffb04f 00 KEY_200+ carMP3.conf
```

Проделываем те же махинации, что и в статье, за исключением содержания irexec.lircrc — его делаем своим. Итак, после перезапуска малинки кнопка пульта должна была выполнить указанные в irexec.lircrc команды, но ничего опять не получилось. Я даже проверяла логи:

```sh
sudo journalctl -u lircd -n 50 --no-pager
```

Но ошибки, вроде бы, и не было, однако ничего не работало как надо. Поэтому было принято стратегическое решение воспользоваться Arduino для работы с пультом — так оказалось в разы проще.

## ИК-приемник и arduino nano

Итак, перед нами цель — сделать так, чтобы Arduino принимала сигналы с ИК-приёмника и отправляла название кнопки через Serial-порт на малинку.
Готовый скетч для этого уже есть в интернете: [https://newbiely.com/tutorials/arduino-nano/arduino-nano-ir-remote-control](https://newbiely.com/tutorials/arduino-nano/arduino-nano-ir-remote-control). Я его впоследствии немного адаптировала под свои нужды (об этом подробнее позже).
Краткий алгоритм действий:

1. Загружаем скетч на Arduino
2. Проверяем через Serial Monitor корректность отображения нажатий кнопок пульта
3. Подключаем собранную схему к Raspberry Pi (не забываем про USB-кабель)

![circuit_image](images/circuit_image.png)

## Написание кода

### Код для камеры и сервоприводов

В изначальном варианте мы хотели использовать YOLO как модель компьютерного зрения для обнаружения объектов (в частности, людей). Однако малинка работала исключительно медленно, поэтому мы отказались от этой идеи. Вместо этого наша камера детектит объекты синего цвета и, соответственно, движется в их сторону. Для анализа изображений мы используем библиотеку OpenCV. Код с YOLO и наш код лежат в исходниках.

### Как arduino взаимодействует с Raspberry pi

Внутри основной программы создаются два потока выполнения:

1. Первый поток отвечает за:
   - Обработку изображений с камеры
   - Поиск координат центра обнаруженного объекта
   - Управление сервоприводами
2. Второй поток выполняет:
   - Ожидание сигналов через Serial-порт от Arduino
   - Обработку нажатий кнопок пульта

Взаимодействие между потоками организовано через очередь сообщений. Исходный код доступен в файле `src/arduino.py`.

### Запись видео

Для добавления функциональности записи видео мы использовали примеры из официального репозитория [picamera2](https://github.com/raspberrypi/picamera2/tree/main). В частности, был взят за основу скрипт [capture_video.py](https://github.com/raspberrypi/picamera2/blob/main/examples/capture_video.py) и интегрирован в наш проект.

1. Поскольку при настройке автозапуска мы использовали команду `sudo crontab -e` вместо `crontab -e`, все прописанные инструкции выполнялись с правами root.

> На компьютере обычно имеются общесистемный файл ( /etc/crontab ), и индивидуальные файлы ( /var/cron/tabs/<имя-пользователя> ) для пользователей системы. Таким образом, команды в файле будут выполняться с правами этих пользователей ...

Поэтому для сохранения видеофайла необходимо указывать полный путь.

2. Включение/выключение записи мы хотели привязать к определённым кнопкам пульта.

```python
# Some code before
elif((command == "0") and not IS_RECORDING):
    IS_RECORDING = True
    filename = f"/home/pi/images/record_{datetime.now().strftime('%Y%m%d_%H%M%S')}.h264"
    picam2.start_recording(encoder, filename)
elif(command == "1" and IS_RECORDING):
    IS_RECORDING = False
    picam2.stop_recording()
```

Но здесь возникла проблема. По всей видимости, `picam2.stop_recording()` полностью выключает камеру. Вот цитата из мануала:

> We also used the convenient start_recording and stop_recording functions, which start and stop both the encoder and the
camera together. Sometimes it can be useful to separate these two operations, for example you might want to start and
stop a recording multiple times while leaving the camera running throughout. For this reason, start_recording could have
been replaced by:

```python
encoder.output = 'test.h264'
picam2.start_encoder(encoder)
picam2.start()
```

> and stop_recording by:

```python
picam2.stop()
picam2.stop_encoder()
```

В таком случае я попыталась использовать `picam2.stop_encoder()` и `picam2.start_encoder()`, но код всё равно не работал. Поэтому я просто добавила `picam2.start()` после `picam2.stop_recording()` — и код заработал.

## Включение камеры по пульту

Теперь необходимо реализовать включение камеры (запуск Python-скрипта) по нажатию кнопки на пульте. Для этого мы можем использовать следующий подход:

1. Arduino при старте подаёт высокий сигнал на определённый GPIO-пин
   - Для этого потребуется модификация существующего скетча
2. Raspberry Pi должен:
   - Детектировать этот сигнал через GPIO
   - Запускать основной Python-скрипт обработки

Основные технические сложности реализации:

1. Требуется дополнительный скрипт, который должен:
   - Автозапускаться при включении Raspberry Pi
   - Ожидать сигнал на GPIO-пине
2. Необходима автоматическая активация демона pigpiod при старте системы:
   - Для корректной работы библиотеки pigpio

Это все довольно легко решается с помощью [Cron](https://timeweb.com/ru/community/articles/chto-takoe-cron).

```sh
sudo crontab -e
```

Добавляем следующую строчку в конец и перезапускаемся:

```sh
@reboot /home/pi/env/bin/python3 /home/pi/listener.py > log_listener.txt 2>&1
```

где listener.py - скрипт, ожидающий сигнала. Теперь при включении малинки, запускается listener.py на заднем фоне.  Это можно проверить командой:

```sh
ps aux | grep python
```

При получении сигнала listener.py запускает еще один скрипт - awake.sh, который лишь запускает демона pigpio и основную прогу.

# Создание корпуса и работа с 3D принтером

Главной задачей было обеспечить подвижность камеры во всех направлениях и добиться органичного расположения всех компонент сборки в пространстве. За счёт отсутствия некоторых особенностей камеры и raspberry pi в 3D моделях из интернета, некоторые измерения были произведены вручную и отражены на 3D деталях, что приводило к неточностям в сборке. В итоге, благодаря клею и напильникам, все вылившиеся в физической форме недочёты были устранены и отлажены под оборудование.

Главной проблемой была разработка надёжного сцепления крепежа и деталей, т. к. винты, предназначенные для сервоприводов и камеры, либо отсутствовали, либо не соответствовали нашим требованиям (ненадёжно вкручивались в пластик за счёт своей малой длины). Эта проблема была решена за счёт подбора размеров и форм 3D деталей, обеспечивающих большую площадь соприкосновения и удерживающих конструкцию с помощью силы трения.

Итоговая конструкция позволяет реализовывать все запрограммированные функции системы и обеспечивает надёжную связку и свободную эксплуатацию всех компонент.

# Перспективы развития и возможные улучшения

Несмотря на работоспособность текущего решения, эти доработки помогут сделать систему более удобной и надежной:

## 1. Заменить использование Cron на systemd для автозапуска скрипта.
И вот почему:
   - Cron запускает задачи строго по расписанию, даже если система не готова (например, сеть не поднята, сервисы не инициализированы). В то время как systemd может ждать завершения загрузки (After=network.target), монтирования дисков или других сервисов, что критично для скриптов, зависящих от окружения.
   - Если процесс упадёт, Cron он не перезапустится. Systemd может перезапускать упавшие процессы (Restart=on-failure), логировать вывод (StandardOutput=journal) и отслеживать их состояние.
   - Cron требует указания пользователя (@reboot pi /path/to/script), а systemd может работать как системный сервис.
Можно добавить файл `/etc/systemd/system/my_script.service` с содержанием:

```sh
[Unit]
Description=My Custom Camera Startup Script
After=network.target

[Service]
ExecStart=/home/pi/env/bin/python3 /home/pi/listener.py
Restart=on-failure
User=pi

[Install]
WantedBy=multi-user.target
```

и активировать его:

```sh
sudo systemctl enable my_script
sudo systemctl start my_script
```

## 2. Исправить шатание камеры
На данном этапе код работает нормально, однако существуют некоторые нюансы. При тестировании устройства мы заметили, что иногда камера начинает шататься из стороны в сторону, хотя синий объект остается неподвижным.

Проанализировав явление, я пришла к выводу, что причиной был свет, отраженный от гладкой поверхности синего пластика. Дело в том, что менялся оттенок синего.
Под капотом (в коде) камера ищет контуры синего цвета, определяет наибольший контур и центрируется относительно него.
То есть из-за бликов оттенок синего постоянно меняется, поэтому один большой синий контур разбивается на несколько. Из-за этого камера начинает "елозить".
Поэтому в планах — придумать решение описанной проблемы.

## 3. Обработка ошибок при нажатии на кнопку пульта
У Arduino не всегда получается распознать кнопку пульта с первого раза (выводится "undefined command"), что особенно критично для:

- Кнопки включения/выключения
- Кнопки запуска/остановки записи видео

В качестве решения предлагается добавить светодиодный индикатор, который будет визуально подтверждать:

 1. Факт приёма сигнала от пульта
 2. Корректность распознавания команды
 3. Текущий статус выполнения (например, мигание при обработке)

## 4. Обработать двойное нажатия кнопки запуска камеры
При включении Raspberry Pi запускается скрипт `listener.py`, ожидающий сигналов с пульта. При обнаружении нужного сигнала запускается основная программа. Однако при повторном нажатии на ту же кнопку (когда программа уже запущена) возникают конфликты.

Дело в том, что `listener.py` запускает ещё один процесс с этой программой, обращающийся к тем же Serial-портам. Это приводит к зависанию обоих запущенных процессов.

В качестве решения проблемы я добавила в `listener.py` проверку на то, что программа уже запущена. Если это так, то нажатие кнопки игнорируется. Однако при таком подходе приходится перебирать все запущенные программы в цикле и искать нужную по имени.

На эти операции уходит много времени, и задержка становится заметной для пользователя. Поэтому в планах — придумать более элегантное решение, которое не требовало бы таких временных затрат.

## 5. Использование оставшихся кнопок пульта
В текущей реализации остаются незадействованные кнопки пульта, которые можно использовать для новых функций. Например, можно устанавливать цвет, который необходимо отслеживать.

## 6. Поиск человека
Возвращение к изначальной цели — реакции на движение человека. Скорее всего, существуют способы улучшения производительности при использовании модели на Raspberry Pi. Например, передача данных напрямую по DMA, снимая нагрузку с CPU.
