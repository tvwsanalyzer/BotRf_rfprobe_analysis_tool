RfProbe: an RF signal path loss and terrain analysis tool 
===========================

**RfProbe:**  is a fork of [Splat! Tool](http://www.qsl.net/kd2bd/splat.html).
Compared to the original Splat! program, RfProbe incorporates several changes, to add more features and allow its execution in a linux web server.

RfProbe is a part of BotRf, tool for the electromagnetic spectrum analysis of Terrain, loss, and RF Signal Propagation that run on [Telegram Messenger](https://telegram.org/) 

BotRf is a multi-platform tool: 
it run on PC or smartphone where can be installed a [*"Telegram Client"* (Windows, Linux, Mac, Android ...)](https://telegram.org/apps) , or in general by connecting to Telegram via web.

BotRf is composed of two parts:

- **rfprobe:** , this tool
- **splatbot.py**: 
	- python program, is the telegram bot interface manager. It uses the ["nickoala telepot"](https://github.com/nickoala/telepot), a Python framework for Telegram.

The splatbot.py program analyzes the data and commands that the user sets in Telegram and passes them to a remote server, where rfprobe is executed. It analyzes them and generate graphics and reports. 
The analysis results are returned to the user through the Telegram interface.
 
## Compile rfprobe

RfProbe is a project consisting of many modules in C/C++ and requires two libraries to be installed in the development environment:
- the bz2 compression library
- the sqlite3 library

To compile the project is available the script **build.sh**: it calls g++ (the GNU C++ compiler) providing the correct options and generates the executable program.

## People who have contributed to the project: 

* Marco Zennaro - ICTP, Guglielmo Marconi Wireless Laboratory (http://wireless.ictp.it/)
* Ermanno Pietrosemoli - ICTP, Guglielmo Marconi Wireless Laboratory (http://wireless.ictp.it/)
* Marco Rainone - SolviTech (https://www.linkedin.com/in/marcorainone) (marcorainone@gmail.com)

