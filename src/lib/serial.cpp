/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 ThundeRatz

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#define _DEFAULT_SOURCE
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#define OPTIONS "b:d:h"
#define nitens(list) 1 [&list] - list
#define try(cmd, msg) do { if ((cmd) == -1) { std::perror(msg); return -1; } } while (0)  // NOLINT(whitespace/braces)

// Ver man 3 termios para as flags para terminais

/*
 * Possíveis constantes de baud:
 * B0
 * B50
 * B75
 * B110
 * B134
 * B150
 * B200
 * B300
 * B600
 * B1200
 * B1800
 * B2400
 * B4800
 * B9600
 * B19200
 * B38400
 * B57600
 * B115200
 * B230400
 * Tem valores consecutivos (http://www.delorie.com/djgpp/doc/incs/termios.h). Uma busca binária em uma array com os
 * valores retorna o valor certo da constante.
 */

static const unsigned int baud_list[] =
{ 0,    50,   75,   110,  134,   150,   200,   300,    600,   1200,
  1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400
};

static int cmp_uints(const void *cmp1, const void *cmp2)
{
  const unsigned int *n1 = reinterpret_cast<const unsigned int *>(cmp1);
  const unsigned int *n2 = reinterpret_cast<const unsigned int *>(cmp2);
  return *n1 - *n2;
}

inline static int baud_bsearch(const int *baud)
{
  unsigned int *found = NULL;

  found = (unsigned int *)bsearch(baud, &baud_list, nitens(baud_list), sizeof(baud_list[0]), cmp_uints);
  if (!found)
    return -1;
  return found - baud_list;
}

/***********************************************************************
 * serial_open
 * Abre um dispositivo serial. O driver para terminais do Linux suporta
 * várias flags e configurações, essa função abre:
 * -Com baud de entrada e saída iguais
 * -Sem checagem de paridade
 * -Sem processamento de saída ou geração de sinais
 * -Transmissão de 8 bits
 * -Modo canônico
 **********************************************************************/
int serial_open(char *dev, const int *baud, int flags)
{
  struct termios tty;
  int fd, speed;

  // Baud de entrada/saída
  if ((speed = baud_bsearch(baud)) == -1)
  {
    fprintf(stderr, "serial_open - Invalid baud rate\n");
    return -1;
  }

  try
    (fd = open(dev, flags | O_NOCTTY), "serial_open - open");

  memset(&tty, 0, sizeof(tty));
  cfsetospeed(&tty, (speed_t)speed);
  cfsetispeed(&tty, (speed_t)speed);

  // Sem paridade
  tty.c_iflag = IGNPAR;

  // Sem processamento adicional da saída
  // (ver manual, há flags para conversão entre convenções como NL -> CRNL automáticas)
  tty.c_oflag = 0;

  // 8 bits de dados, mas a especificação diz que o bit 7 é sempre 0 para NMEA
  tty.c_cflag |= (CS8 | CLOCAL | CREAD);

  // VMIN e VTIME setam número mínimo de caracteres para ler antes de retornar read
  // e timeout

  // Modo canônico
  tty.c_lflag = ICANON;
  tcflush(fd, TCIFLUSH);
  try
    (tcsetattr(fd, TCSAFLUSH, &tty), "serial_open - tcsetattr");
  return fd;
}
