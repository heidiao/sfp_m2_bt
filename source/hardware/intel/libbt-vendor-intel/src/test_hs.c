/**********************************************************************
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 **********************************************************************/

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <stdlib.h>
#include <sys/ioctl.h>

/*******************************************************************************
**
** Function          open_ftdi_port
**
** Description       Open FTDI port and do the configurations.
**
** Returns           port : file descriptor value of the device openend
**
*******************************************************************************/
int open_ftdi_port(char* device, char* speed)
{
    struct termios g_initialAtt, newAtt;
    int port = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

    int ispeed = atoi(speed); //convert string to int //

    if (port == -1)
    {
        printf("ERROR port -1");
        exit(0);
    }

    printf("User specified speed as: %d \n",ispeed);

    printf("+%s\n",__func__);
    tcgetattr(port,&g_initialAtt);// save this to restore later
    newAtt=g_initialAtt;

    if(ispeed == 921600)
    {
        newAtt.c_cflag = B921600 | CS8 | CLOCAL | CRTSCTS; // Enable RTS CTS flow control
    } else if(ispeed == 2000000)
    {
        newAtt.c_cflag = B2000000 | CS8 | CLOCAL | CRTSCTS; // Enable RTS CTS flow control
    } else if(ispeed == 3000000)
    {
        newAtt.c_cflag = B3000000 | CS8 | CLOCAL | CRTSCTS; // Enable RTS CTS flow control
    }
    else if(ispeed == 115200)
    {
        newAtt.c_cflag = B115200 | CS8 | CLOCAL | CRTSCTS; // Enable RTS CTS flow control
    } else //default rate: 115200
    {
        newAtt.c_cflag = B115200 | CS8 | CLOCAL | CRTSCTS; // Enable RTS CTS flow control
    }
    cfmakeraw(&newAtt);
    tcsetattr(port,TCSANOW,&newAtt);
    return port;
}

/*******************************************************************************
**
** Function          char_to_hex
**
** Description       coverting character in to hexadecimal value.
**
** Returns           hex value
**
*******************************************************************************/
unsigned char char_to_hex(char c)
{
    if (c == '0')
        return 0x00;
    else if (c == '1')
        return 0x01;
    else if (c == '2')
        return 0x02;
    else if (c == '3')
        return 0x03;
    else if (c == '4')
        return 0x04;
    else if (c == '5')
        return 0x05;
    else if (c == '6')
        return 0x06;
    else if (c == '7')
        return 0x07;
    else if (c == '8')
        return 0x08;
    else if (c == '9')
        return 0x09;
    else if (c == 'A' || c == 'a')
        return 0x0a;
    else if (c == 'B' || c == 'b')
        return 0x0b;
    else if (c == 'C' || c == 'c')
        return 0x0c;
    else if (c == 'D' || c == 'd')
        return 0x0d;
    else if (c == 'E' || c == 'e')
        return 0x0e;
    else if (c == 'F' || c == 'f')
        return 0x0f;
    else
        return -1;
}

/*******************************************************************************
**
** Function          byte2int
**
** Description       coverting hexadecimal character to integer.
**
** Returns           integer value else return -1 for non hexadecimal value
**
*******************************************************************************/
int byte2int(unsigned char c)
{
    printf("byte2int: %c\n", c);
    switch (c)
    {
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '0':
            return c-'0';
        case 'a':
        case 'A':
            return 10;
        case 'b':
        case 'B':
            return 11;
        case 'c':
        case 'C':
            return 12;
        case 'd':
        case 'D':
            return 13;
        case 'e':
        case 'E':
            return 14;
        case 'f':
        case 'F':
            return 15;
        default:
            return -1;
    }
}

/*******************************************************************************
**
** Function          parse_cmd
**
** Description       Parse the command read from file and send to lower layer.
**
** Returns           None
**
*******************************************************************************/
void parse_cmd(int fd, char* cmd_str)
{
    struct timeval timeout;
    int n;
    unsigned int i;
    unsigned char c;
    // initialise the timeout structure
    timeout.tv_sec = 1; // ten second timeout
    timeout.tv_usec = 0;

    size_t cmd_size = strlen(cmd_str);
    for(i=0; i<cmd_size; i++)
    {
        if (i%2 == 1)
        {
            c |= char_to_hex(cmd_str[i]);
            printf(" %x ",c);
            write(fd, &c, 1);
        }
        else
        {
            c = char_to_hex(cmd_str[i]) << 4;
        }
    }

    //printf("Event: \n");
    //unsigned char buf[40];
    char bc[2]={'\0'};
    int event_started = 0;
    unsigned int cc = 0, event_code = 0;
    unsigned int len = 0;
    char x = '\0';
    //bzero(buf,40);
    i = 0;
    //sleep(1);
    do{
        //n = read(fd, &buf[i], 1);
        n = read(fd, bc, 1);
        //val = atoi(&buf[i]);
        sscanf(&bc[0], "%c", &x);

        if (x == 4 && event_started == 0)
        {
            printf("\n\t>> %x ",bc[0]);
            event_started = 1;
            cc = 0;
        }
        else if (event_started == 1)
        {
            cc++;
            if (cc == 1)
            {
                event_code = x;
                if (x == 15) //Debug stratup event. Skip this event
                {
                    printf ("Debug Startup event\n");
                    break;
                }
                printf (" %x ", bc[0]); //Event code
            }
            else if (cc == 2)
            {
                printf(" %x ",bc[0]);
                len = x;
            }
            else if (cc >= 2) {
                printf(" %x ", bc[0]);
                len--;
            }
            if (len <= 0 && cc >= 2)
                break;
        }
        i++;
    }while(1);

    if (event_code == 14)
        printf (" (Command complete)\n");
    // check if an error has occured
    if(n < 0)
    {
        perror("select failed\n");
    }
    else if (n == 0)
    {
        puts("Timeout!");
    }
    else
    {
        //printf("\nBytes detected on the port!\n");
        printf ("\n");
        return;
    }
}

/*******************************************************************************
**
** Function          open_file
**
** Description       Open the test file and execute commands one by one.
**
** Returns           None
**
*******************************************************************************/
void open_file(int fd, char* filename)
{
    FILE * fp;
    char line[1024];
    size_t len = 0;
    ssize_t read;
    char* ret = NULL;
    fp = fopen(filename, "r");
    if (fp == NULL)
       exit(1);

    while (1) {
       ret = fgets (line, sizeof (line), fp);
       if (ret == NULL || strcmp(ret,"") == 0 || strlen(line) <= 0)
               break;
       line[strlen(line)-1] = '\0';
       if (line[0] == '#') //comment line. Print it
       {
               printf("%s\n", line);
               continue;
        }
       //printf(":%s:\n", line);
       parse_cmd(fd, line);
       ret = NULL;
    }
    fclose(fp);
}

int main(int argc, char* argv[])
{
    unsigned char* cmd;
    printf("OPEN PORT\n");
    int fd = -1;

    //Check if user passes enough arguments //
    if (argc > 3)
    {
         if(fd == -1)
        {
            fd = open_ftdi_port(argv[1],argv[3]);
            printf("FILE:%s\n",argv[2]);
            open_file(fd, argv[2]);
        }
    }
    else
        printf(">>> Invalid args. Usage: exe <serial_port> <file_name> <speed> <<< \n");

    if(fd != -1)
      close(fd);

    return(0);

}
