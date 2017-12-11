#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "defs.h"
#include "choose_generator_matrix.h"
//#include "Gen_shift_value.h"


   short *ldpc_encoder_header(short *test_input,short block_length,double rate)
	{
	   printf("ldpc encoder %d\n", test_input[0]);
	   short *c; //padded codeword
	   short *channel_input;	//output sequence
		
		short *Gen_shift_values, *no_shift_values, *pointer_shift_values;
		short BG,Zc,Kb,nrows,ncols;
		short channel_temp;
		int i1,i2,i3,i4,i5,t,temp,temp_prime;
		int no_punctured_columns;  

		//Table of possible lifting sizes
		short lift_size[51]={2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,18,20,22,24,26,28,30,32,36,40,44,48,52,56,60,64,72,80,88,96,104,112,120,128,144,160,176,192,208,224,240,256,288,320,352,384};
	
		 //determine number of bits in codeword
		 //if (block_length>3840)
		  //{
			  BG=1;
			  Kb = 22;
			  nrows=46;	//parity check bits
			//  ncols=22;	//info bits
		  // }
		 /*else if (block_length<=3840)
		   {
			  BG=2;
		      nrows=42;	//parity check bits
			  ncols=10;	// info bits
			  if (block_length>640)
				 Kb = 10;
			  else if (block_length>560)
				 Kb = 9;
			  else if (block_length>192)
				 Kb = 8;
			  else
				 Kb = 6;
		   }
*/
		 //find minimum value in all sets of lifting size
		 for (i1=0; i1 < 51; i1++)
		   {
			  if (lift_size[i1] >= (double) block_length/Kb)
				{
				  Zc = lift_size[i1];
				   //printf("%d\n",Zc);
				  break;
				}
		   }

		 // load base graph of generator matrix
		 if (BG==1)
		   {
			  no_shift_values=(short*) no_shift_values_BG1;
			  pointer_shift_values=(short*) pointer_shift_values_BG1;          
		   }

		 else if (BG==2)
		   {       
		   no_shift_values=(short*) no_shift_values_BG2;
		   pointer_shift_values=(short*) pointer_shift_values_BG2;
			   
		   }
		 Gen_shift_values=choose_generator_matrix(BG,Zc);
		 no_punctured_columns=(int)((nrows+Kb-2)*Zc-block_length/rate)/Zc; 
		//printf("%d\n",no_punctured_columns);
		
		//padded input sequence  
		c=(short *)malloc(sizeof(short) * Kb * Zc);
		channel_input  = (short *)malloc( (Kb+nrows-no_punctured_columns) * Zc *sizeof(short));
		memset(c,0,sizeof(short) * Kb * Zc);
		memcpy(c,test_input,block_length * sizeof(short));
		
		// parity check part
		 for (i2=0; i2 < Zc; i2++)
		   {
			  t=Kb*Zc+i2;
			  //rotate matrix here
		  for (i5=0; i5 < Kb; i5++)
			{
			   temp = c[i5*Zc];
			   memmove(&c[i5*Zc], &c[i5*Zc+1], (Zc-1)*sizeof(short));
			   c[i5*Zc+Zc-1] = temp;           
			}   
			 
			  // calculate each row in base graph
			  for (i1=0; i1 < nrows-no_punctured_columns; i1++) 
			   {
			   channel_temp=0;
			   for (i3=0; i3 < Kb; i3++)
					 {
				temp_prime=i1 * ncols + i3;
						for (i4=0; i4 < no_shift_values[temp_prime]; i4++)
						  {
							 channel_temp = channel_temp ^ c[ i3*Zc + Gen_shift_values[ pointer_shift_values[temp_prime]+i4 ] ];
						  }
					 }

			   channel_input[t+i1*Zc]=channel_temp;
			
			   }
		   }

		// information part
		memcpy(channel_input,c,Kb*Zc*sizeof(short));
		
		return channel_input;
	}
