/*
Подключение к МК

	- Кнопка, подтянутая к питанию резистором. Замыкается на землю, формируя низкий уровень на пине.
	- Кнопка Ресет замыкает пин-ресет на землю
	- Светик желтый (анод-катод на пинах)
	- Светик красный - имитатор затвора мосфета (один конец на земле, другой на пине)

 */

//#define F_CPU 4800000UL/8UL	// Рабочая частота, пониженное энергопотребление
#define F_CPU 4800000UL		// для отладки, чтобы быстрее прошивалось

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define LED_DDR		DDRB
#define LED_PORT	PORTB
#define LED_PIN		PINB
#define LED_CATHODE PB4		// |подключение светика-сенсора. Он же индикатор работы и выбранных режимов.|
#define LED_ANODE	PB0		// |------------------------------------------------------------------------|

#define MOTOR_DDR			DDRB
#define MOTOR_PORT			PORTB
#define MOTOR_PIN			PINB
#define MOTOR_PIN_NUMBER	PB2		// для отладки будет анодом красного светика, так как рядом с VCC и не надо далеко тянуться. На пшике - гаснет.

#define BUTTON_DDR			DDRB
#define BUTTON_PORT			PORTB
#define BUTTON_PIN			PINB
#define BUTTON_PIN_NUMBER	PB1		// кнопка

#define LED_CATHODE_HIGH	{ LED_PORT |= (1 << LED_CATHODE); }
#define LED_CATHODE_LOW		{ LED_PORT &= ~(1 << LED_CATHODE); }

#define LED_ANODE_HIGH		{ LED_PORT |= (1 << LED_ANODE); }
#define LED_ANODE_LOW		{ LED_PORT &= ~(1 << LED_ANODE); }



#define CALIBRATE 5				// Номер режима для калибровки
#define DEBUG 1					// Режим отладки: 1 - вкл, 0 - выкл
#define DEBUG_OUTPUT 1			// Если включено, то включается функция дебаг-вывода-моргания светиком.
								// Моргается 16 бит переданной в функцию переменной.
								// Можно передать и 8-битную переменную. Тогда первые 8 бит будут нулями.
								// Пришлось извратиться в отсутствие УАРТ на тиньке для отладки
#define MOTOR_PULSE_LENGTH 100	// Длительность работы моторчика в мс (х10)

#if (DEBUG)
#define LIGHT_ON_SETTING	5	// сколько опросов (1 опрос в 4 секунды) должен гореть свет для того чтобы пшикнуло (5 * 4 = 20 секунд)
#else
#define LIGHT_ON_SETTING	45	// сколько опросов (1 опрос в 4 секунды) должен гореть свет для того чтобы пшикнуло (45 * 4 = 180 секунд)
#endif

void wd_on();
void wd_off();
void blink(uint8_t, uint8_t, uint8_t);
void delay_ms(uint8_t);
void pshik();
void red_blink();
uint8_t measure();
void calibrate();
void blink_word(uint16_t);


uint8_t  EEMEM luminocity_threshold  = 1;	// Порог уровня темноты. Все, что дольше (порога + 3) = темно. Дефолтное значение для желтого непрозрачного светика.
uint16_t EEMEM pshik_timeout_setting = 10;	// Установка интервала пшиков. Дебаг - 10 (~40 секунд)
int16_t pshik_timeout_counter;				// Счетчик интервала пшиков уменьшается в прерывании watchdog каждые 4 секунды.
uint8_t light_on_flag = 0;					// Флаг устанавливается, если при замере свет включен и сбрасывается, если выключен
uint8_t light_on_counter = 0;				// Счетчик опросов для времени пока включен свет
uint8_t timeout_pshik_flag = 1;						// Флаг вкл-выкл пшикания по таймауту

ISR (INT0_vect){
	// Обработка кнопки
	asm("cli");
	blink(1, 100, 50);	// антидребезг (имеет ли смысл в данном приложении?..) и порог для длинного нажатия, вход в "меню" настройки. Одна длинная вспышка.

	uint8_t mode = 0;	// Режим

	while (!(BUTTON_PIN & (1 << BUTTON_PIN_NUMBER))){	// если кнопка отпущена, то выходим из цикла.
		if(mode == CALIBRATE) break;	// Если додержались до режима калибровки, насильный выход из цикла. Дальше этого режима цикл не уйдет.
		mode++;							// Следующий режим
		blink(mode, 10, 30);			// Индикация выбранного режима, кнопка до сих пор нажата
		delay_ms(50);
	}

#if (DEBUG)
	switch (mode){
	case 0: {pshik(); break;}								// Если было одно короткое нажатие - предупреждающе мигаем и пшикаем
	case 1: {timeout_pshik_flag = 1; eeprom_update_word(&pshik_timeout_setting, 1); pshik_timeout_counter = 1; break;}	// Установка интервала пшиков в 4 c (дебаг)
	case 2: {timeout_pshik_flag = 1; eeprom_update_word(&pshik_timeout_setting, 3); pshik_timeout_counter = 3; break;}	// Установка интервала пшиков в 12 с (дебаг)
	case 3: {blink_word((uint16_t)eeprom_read_byte(&luminocity_threshold)); break;}								//  дебаг
	case 4: {timeout_pshik_flag = 0; break;}						// Установка не пшикать по таймауту
	case CALIBRATE: {calibrate(); break;}					// Калибровка
	}
#else
	switch (mode){
	case 0: {pshik(); break;}						// Если было одно короткое нажатие - предупреждающе мигаем и пшикаем
	case 1: {timeout_pshik_flag = 1; eeprom_update_word(&pshik_timeout_setting, 900);  pshik_timeout_counter = 900;  break;}	// Установка интервала пшиков в 1 час  (3600 с \ 4 с = 900)
	case 2: {timeout_pshik_flag = 1; eeprom_update_word(&pshik_timeout_setting, 1800); pshik_timeout_counter = 1800; break;}	// Установка интервала пшиков в 2 часа (7200 с \ 4 с = 1800)
	case 3: {timeout_pshik_flag = 1; eeprom_update_word(&pshik_timeout_setting, 2700); pshik_timeout_counter = 2700; break;}	// Установка интервала пшиков в 3 часа (10800 с \ 4 с = 2700)
	case 4: {timeout_pshik_flag = 0; break;}				// Установка не пшикать по таймауту
	case CALIBRATE: {calibrate(); break;}			// Калибровка
	}
#endif

	blink(mode, 5, 15);	// Подтверждаем режим более быстрым миганием соответствующее количество раз.
	asm("sei");
}

ISR (WDT_vect){
	if(light_on_flag){		// если включен свет,
		light_on_counter++;	// то тикаем счетчиком
	}
	else{					// когда свет выключен или только что потух,
		if (light_on_counter >= LIGHT_ON_SETTING){	// смотрим, горел ли он достаточно долго,
			light_on_counter = 0;					// сбрасываем счетчик длительности включенного света
			int16_t temp = eeprom_read_word(&pshik_timeout_setting);
			pshik_timeout_counter = temp;			// счетчик таймаута пшиков,
			pshik();								// и пшикаем.
		}
		if(timeout_pshik_flag) pshik_timeout_counter--;	// А если свет горел недостаточно долго, то продолжаем тикать таймером таймаута, если таймаут-пшик-флаг устновлен
	}
}


void wd_off(){
	asm("wdr");
	WDTCR |= (1 << WDCE) | (1 << WDE);
	WDTCR = 0x00;
}

void wd_on(){
	WDTCR |= (1 << WDTIE) | (1 << WDCE);
	WDTCR |= (1 << WDP3);	// переполнение каждые 4 с
}

void calibrate(){
//	blink(4, 5, 10); blink(1, 50, 5);		// Индикация начала калибровки

	eeprom_update_byte(&luminocity_threshold, measure());	// калиброва и обновление переменной порога освещенности. Свет должен быть включен.

//	blink(4, 5, 10); blink(2, 50, 5);		// Индикация конца калибровки
}

void pshik(){
	blink(15, 5, 3);
	MOTOR_PORT |= (1 << MOTOR_PIN_NUMBER);
	delay_ms(MOTOR_PULSE_LENGTH);
	MOTOR_PORT &= ~(1 << MOTOR_PIN_NUMBER);
}

void red_blink(){	// для дебага моргание светиком, имитирующем моторчик
	MOTOR_PORT |= (1 << MOTOR_PIN_NUMBER);
	delay_ms(10);
	MOTOR_PORT &= ~(1 << MOTOR_PIN_NUMBER);
	delay_ms(10);
}

void delay_ms(uint8_t count){	// переделка функции задержки. Не пойму почему, но без нее не работает.
	while (count--){			// Желаемую задержку в мс делим на 10 для передачи в функцию.
		asm("wdr");				// То есть, задержка в 1000 мс будет delay_ms(100);
		_delay_ms(10);
	}
}


void blink(uint8_t blinks, uint8_t length, uint8_t delay){	// мигаем светиком-сенсором
	uint8_t backup_ddr = LED_DDR;
	uint8_t blink_counter = 0;

	LED_DDR |= (1 << LED_ANODE | 1 << LED_CATHODE); // подготавливаем ножки светика
	LED_ANODE_LOW; LED_CATHODE_LOW;

	while (blink_counter < blinks){
		LED_ANODE_HIGH;		// зажигаем
		delay_ms(length);	// горим
		LED_ANODE_LOW;		// тушим
		delay_ms(delay);	// ждем между миганиями
		blink_counter++;	// Увеличиваем счетчик миганий
	}

	LED_DDR = backup_ddr; // Возвращаем прежние значения
}

uint8_t measure(){
//	blink(4, 10, 5);	// индикация начала измерения
	red_blink();		// дебаг

	LED_DDR |= (1 << LED_ANODE | 1 << LED_CATHODE);		// Пины светика на выход
	LED_ANODE_LOW; LED_CATHODE_HIGH;					// Обратное включение светика. Заряд паразитной емкости.
	asm("nop");asm("nop");asm("nop");
	LED_DDR &= ~(1 << LED_CATHODE); LED_PORT &= ~(1 << LED_CATHODE);		// Изолируем анод от питания, выключаем внутренний подтягивающий резистор.
																			// Пошла разрядка
	uint16_t i = 0;	// переменная для переполнения
	uint8_t Luminocity = 0;	// Количество переполнений переменной i. Пропорционально величине освещенности.

	while ((LED_PIN & (1 << LED_CATHODE)) != 0) {	// Считаем время разрядки до логического нуля
		asm("wdr");
		if(!i++) Luminocity++;
	}

	if (Luminocity < (eeprom_read_byte(&luminocity_threshold) + 5)) {	// 5 это про запас, чтобы
		light_on_flag = 1;	// светло
//		blink(1, 10, 1);
		red_blink(); red_blink();
	}
	else {
		light_on_flag = 0;	// темно
//		blink(3, 10, 5);
		red_blink(); red_blink(); red_blink();	// дебаг
	}

//	blink_word(Luminocity);		// DEBUG_OUTPUT должен быть 1

	return Luminocity;
}

#if (DEBUG_OUTPUT)		// побитовый вывод переменных для дебага. Весит 198 байт (106 байт если без второго байта)
void blink_word(uint16_t word_to_blink){
	red_blink(); red_blink(); red_blink(); // трижды моргаем красным светиком - начало вывода
	delay_ms(200);
	for (int i = 15; i >= 0; i--){
		if (((word_to_blink & (1<<i))>>i)) {red_blink(); blink(1 ,70, 50);}
		else {red_blink(); delay_ms(100);}
	}

//delay_ms(200);
//blink(3,10,10);
//delay_ms(200);
//
//for (int i = 7; i >= 0; i--){
//	blink(1, 5, 50);
//	if (((eeprom_read_byte(&luminocity_threshold) & (1<<i))>>i)) {blink(1 ,70, 50);}
//	else {delay_ms(50);}
//}
	red_blink();red_blink();red_blink();red_blink();	// конец вывода - четыре мигания красным светиком.
}
#endif


int main(){

// Interrupt INT0 init - для кнопки. По нему же МК будет просыпаться при нажатии для пшика или настройки.
	BUTTON_DDR |= (1 << BUTTON_PIN_NUMBER); BUTTON_PORT |= (1 << BUTTON_PIN_NUMBER);
	MCUCR &= ~( (1 << ISC01) | (1 << ISC00)	);	// прерывание по низкому уровню на пине
	GIMSK |= (1 << INT0);

// Motor init
	MOTOR_DDR |= (1 << MOTOR_PIN_NUMBER);	// пин затвора мосфета на выход
	MOTOR_PORT &= ~(1 << MOTOR_PIN_NUMBER);	// закрыт по умолчанию

// Sleep init
	MCUCR |= (1 << SE) | (1 << SM1);
		// SE - Sleep Enable
		// SM[1..0] = 10 - PowerDown sleep mode

// WatchDog init
	wd_on();

	asm("sei");		// Разрешаем прерывания

// Timeout counter init
	int16_t temp = eeprom_read_word(&pshik_timeout_setting);
	pshik_timeout_counter = --temp;

//	calibrate();	// Калибровка при включении. Для дебага. Отпала надобность.
					// luminocity_threshold уже хранится в EEPROM по умолчанию и извлекается по надобности в теле программы
					// Функция calibrate() обновляет ее значение в EEPROM

	delay_ms(100);	// Задержка 1 сек перед стартом основного цикла

	while(1){
		measure();
		if(!pshik_timeout_counter){	// пшик по таймауту
			pshik();
			int16_t temp = eeprom_read_word(&pshik_timeout_setting);	// читаем из eeprom уставку таймаута
			pshik_timeout_counter = temp;								// и восстанавливаем исходное состояние сетчика таймаута
		}
		asm("sleep");		// спим и экономим милливатты батарейки. Просыпаемся каждые 4 секунды от прерывания вотчдога или по нажатию кнопки.
	}

	return 0;
}
