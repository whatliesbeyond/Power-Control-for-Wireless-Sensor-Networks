/*****
 An adaptive power control and routing algorithm for Wireless Sensor Networks (WSN) with limited energy resources.
 This scheme targets on imporving overall lifetime of the WSN by effectively reducing its power consumption and load balancing in the network using two essential schemes:
 A) Transmission Range Adaptation
 B) Cost Adaptation
 (c) 2013, Dewang Lahariya, Dr Asis Nasipuri
 This code and algorithm is available for use and modification under MIT license. Please check license on GitHub.
 *****/

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<time.h>
#include<math.h>

#define INFINITE 99999

int mxNo;

// X,Y coordinates for Wireless Sensor Nodes deployes randomly in the field
struct node
{
    float x_cord;
    float y_cord;
    
};


double D(struct node A, struct node B)
{
    float a,b,c,d;
    a=A.x_cord;
    b=A.y_cord;
    c=B.x_cord;
    d=B.y_cord;
    double dist=sqrt((pow((a-c),2))+(pow((b-d),2)));
    
    return dist;
}


/* Utility function used by shortpath Function to mark selected nodes */
int allselected(int *selected)
{
    
    int iii;
    for(iii=0;iii<mxNo;iii++)
        if(selected[iii]==0)
            return 0;
    return 1;
}

/* A function to the shortest path for transmissions from each node to basestation */

void shortpath(float cost[][mxNo],int *preced,float *distance)
{
    int selected[400];
    
    memset( selected, '\0', sizeof(selected));
    int current=0,ii,kk;
    float newdist,dc,smalldist;
    for(ii=0;ii<mxNo;ii++)
        distance[ii]=INFINITE;
    selected[current]=1;
    distance[0]=0;
    current=0;
    for(int j=allselected(selected);j<mxNo;j++)
    {
        smalldist=INFINITE;
        dc=distance[current];
        for(ii=0;ii<mxNo;ii++)
        {
            if(selected[ii]==0)
            {
                newdist=dc+cost[current][ii];
                
                if(newdist<distance[ii])
                {
                    distance[ii]=newdist;
                    preced[ii]=current;
                }
                if(distance[ii]<smalldist)
                {
                    smalldist=distance[ii];
                    kk=ii;
                }
            }
        }
        current=kk;
        
        selected[current]=1;
        
    }
    
}



int main()
{
    FILE *fp,*fp1,*fp2,*fp3,*fp4;
    fp=fopen("/Users/dewanglahariya/Documents/Simulation_result3.txt","w");
    fp1=fopen("/Users/dewanglahariya/Documents/Simulation_result3.1.txt","w");
    fp2=fopen("/Users/dewanglahariya/Documents/Simulation_result3.2.txt","w");
    fp3=fopen("/Users/dewanglahariya/Documents/Simulation_result3.3.txt","w");
    fp4=fopen("/Users/dewanglahariya/Documents/Simulation_result3.4.txt","w");
    
    
    int r,z,l=0;
    unsigned int i,j,k;
    float t=0,u=0;
   	printf("\nenter the no of nodes\n");
    scanf("%d",&mxNo);
    fprintf(fp,"No. of nodes=%d",mxNo);
    fprintf(fp4,"No. of nodes=%d",mxNo);
    printf("enter the range of nodes\n");
    scanf("%d",&r);
    fprintf(fp,"\nRange=%d",r);
    fprintf(fp4,"\nRange=%d",r);
    printf("enter the number of loops\n");
    scanf("\n%d",&l);
    fprintf(fp,"\nNumber of loops= %d",l);
    fprintf(fp4,"\nNumber of loops= %d",l);
    float q=0,s=0;
    int Trui=900; //Route Update Intervals Transmissions
    int Td=900; //Data Packtes Transmissions
    float Irt=0.02, Trt =0.14;
    float Idt=0.02,Tdt=0.14;
    float Irr=0.02,Trr=0.14;
    float Idr=0.02,Tdr=0.14;
    float Is=0.0095,Ts=7;
    float Ip=0.008,Tp=0.003;
    float cap[mxNo];
    memset( cap, 0, sizeof(cap) );
    float lifetime[mxNo];
    memset( lifetime, 0, sizeof(lifetime) );
    float Lmax= 0;
    float Lmin= 0;
    int max=0,min=0;
    
    srand(time(NULL));
    
    
    int p=0;//parent node index
    float To_dp=0;//Total overheard data packets
    float T_dp=0;//Total data packets transmitted
    float avg_dp_trans=0;
    float avg_dp_ovrhrd=0;
    float Savg_dp_trans[l];
    memset( Savg_dp_trans, 0, sizeof(Savg_dp_trans) );
    float Savg_dp_ovrhrd[l];
    memset( Savg_dp_ovrhrd, 0, sizeof(Savg_dp_ovrhrd) );
    float total_cost=0;
    float avg_cost;
    float avg_current=0;
    float I[mxNo];//current consumption per node
    memset( I, 0, sizeof(I) );
    int N[mxNo];//no of neighbors per node
    memset( N, 0, sizeof(N) );
    int Nn[mxNo];
    memset (Nn, 0, sizeof(Nn) );
    int Na[mxNo][mxNo];//Neighbor array
    memset (Na, 0, sizeof(Na) );
    float total_I=0;
    int preced[mxNo]; //predecessor node array
    
    
    
    float cost[mxNo][mxNo]; //An array to keep record of cost between 2 nodes
    memset( cost, 0, sizeof(cost) );
    float dp[mxNo]; //An array to keep record of data packets on each node
    memset( dp, 0, sizeof(dp) );
    float dpn[mxNo]; //An array to keep record of data packets on neighbor node of each node
    memset(dpn, 0, sizeof(dpn));
    float o_dp[mxNo]; //An array to keep record of overheard data packets on each node
    memset(o_dp, 0, sizeof(o_dp));
    float o_dpn[mxNo]; //An array to keep record of overheard data packets on neighbor node
    memset(o_dpn, 0, sizeof(o_dpn));
    
    double range[mxNo];//An array to keep record of Transmission Range for each node
    
    struct node n[mxNo];//stores node locations
    
    
    for(i=0;i<mxNo;i++)
    {
        float x;
        float y;
        x =((float)rand()/(float)RAND_MAX) * 1000;//gives random x axis location for node
        y = ((float)rand()/(float)RAND_MAX) * 400;//gives random y axis location for node
        cap[i]= ((float)rand()/(float)RAND_MAX) * 50 + 50;
        fprintf(fp," \nn[%u]  x=%.2f   y=%.2f",i,x,y);
        n[i].x_cord=x;
        n[i].y_cord=y;
        range[i]=r;
        //fprintf(fp,"Range= %f",range[i]);
        
    }
    //Assigning costs to nodes
    for(i=0;i<mxNo;i++)
    {
        
        for(k=0;k<mxNo;k++)
        {
            if(i!=k)
            {
                dp[i]=1;
                //Trange[k] = D(n[i],n[k]);
                if(D(n[i],n[k])<=range[k])
                {
                    cost[i][k]=1.0;
                    
                    Na[i][k]=k;
                    fprintf(fp3,"\nNa[%d][%d]=%d",i+1,k+1,Na[i][k]+1);
                    N[i]++;
                }
                else if(D(n[i],n[k])>range[k])
                {
                    cost[i][k]=INFINITE;
                    Na[i][k]=0;
                    fprintf(fp3,"\nNa[%d][%d]=%d",i+1,k+1,Na[i][k]);
                }
                
                
                
            }
            fprintf(fp1,"\ncost[%d][%d]=%f",i+1,k+1,cost[i][k]);
            
        }
        
        //fprintf(fp,"\n No of neighbors %d",N[i]);
    }
    preced[mxNo];
    memset ( preced, 0 , sizeof (preced));
    float distance[mxNo];
    
    //Shortpath function called
    shortpath(cost,preced,distance);
    
    for(i=0;i<mxNo;i++)
    {
        memcpy(&Nn[i],&N[i],sizeof Nn);
        printf("\n Node %d, Predecessor= %d, Total cost= %f ",i+1,preced[i]+1,distance[i]);
        fprintf(fp, "\nNode %d, Predecessor= %d, Total cost= %f ",i+1,preced[i]+1,distance[i]);
        // total_cost=total_cost+distance[i];//sum of all cost
    }
    
			 
    printf("\n\n");
    fprintf(fp,"\n\n");
    
    for(i=0;i<mxNo;i++)
    {
        // dp[i]=1;
        k=i;
        for(j=0;j<mxNo;j++)
        {
            p=preced[k];
            dp[p]++;
            if(p==0)	break;
				        k=p;
            
				   	}
    }
    
    for(i=0;i<mxNo;i++)
    {
        for(k=0;k<mxNo;k++)
        {
            if(i!=k)
            {
                if(D(n[i],n[k])<=range[k])
                {
                    o_dp[i]=o_dp[i]+dp[k];
                    
                }
                
            }
        }
        
								To_dp=To_dp+o_dp[i];
								
    }
    
    //Calculating lifetime at each node
    for(p=0;p<mxNo;p++)
    {
				    //printf(" \nNode %d Tr.data packets= %d",p+1,dp[p],dp[p]-1);
				    //fprintf(fp," \nNode %d Tr.data packets= %f",p+1,dp[p]);
        I[p]=(((Irt*Trt)/Trui)+(((dp[p])*Idt*Tdt)/Td)+(Nn[p]*(((Irr*Trr)/Trui)+(o_dp[p]*(Idr*Tdr)/Td)))+((Is*Ts)/Td)+(8*Ip*Tp));
        /***** In above formula Is*Ts/Td or Tdt******/
        // printf("\nnode %d I= %f",p+1,I[p]);
							 fprintf(fp,"\nnode %d Neigh= %d",p+1,Nn[p]);
							 lifetime[p]=cap[p]/(I[p]*10);
							 fprintf(fp,"\nnode %d I= %f capacity= %f lifetime= %f",p+1,I[p],cap[p],lifetime[p]);
        total_I=total_I+I[p];
    }
    
    
    Lmax = lifetime[0];
    Lmin = lifetime[0];
    
    for(i=0;i<mxNo;i++)
    {
						 	if(Lmax<lifetime[i])
                            {
                                Lmax=lifetime[i];
                                max=i;
                            }
							 if(Lmin>lifetime[i])
                             {
                                 Lmin=lifetime[i];
                                 min=i;
                             }
    }
    printf("\nLmax = %f",Lmax);
    fprintf(fp,"\nLmax = %f node= %d",Lmax,max+1);
    printf("\nLmin = %f",Lmin);
    fprintf(fp,"\nLmin = %f node= %d",Lmin,min+1);
    
    fprintf(fp,"\n Total overheard datapackets= %.2f",To_dp);
    avg_dp_ovrhrd=To_dp/mxNo;
    fprintf(fp,"\nAvg data packets overheard %.2f",avg_dp_ovrhrd);
    fprintf(fp4,"\nLmax = %f node= %d",Lmax,max+1);
    fprintf(fp4,"\nLmin = %f node= %d",Lmin,min+1);
    
    
    fprintf(fp4,"\nAvg data packets overheard %.2f",avg_dp_ovrhrd);
    //Picking up the node with Minimum Lifetime and Running Transmission Range Adaptation Scheme for (Number of loops entered by the user/2), this will boost overall network lifetime.
    for(z=0;z<l/2;z++)
    {
        fprintf(fp2,"\n\n");
        fprintf(fp4,"\n\n");
        fprintf(fp3,"\n\n");
        fprintf(fp4,"\n Tr range adaptation loops");
        fprintf(fp4,"\nLoop No= %d",z+1);
        fprintf(fp3,"\nLoop No= %d",z+1);
        total_cost=0;
        int w;
        To_dp=0;
	       o_dpn[mxNo];
	       memset (o_dpn,0,sizeof(o_dpn));
           Nn[mxNo];
	       memset (Nn,0,sizeof(Nn));
	       int h;
           int g;
				 	  for(i=0;i<mxNo;i++)
                      {
                          if(i==min)//0.2*((Lmax+Lmin)/8)
                          {
                              w=i;
                              fprintf(fp2,"\nNode %d Neighbors= ",i+1);
                              
                              for(k=0;k<mxNo;k++)
                              {
                                  if(Na[i][k]!=0)
                                  {
                                      fprintf(fp2," %d",k+1);
                                      if(D(n[k],n[preced[k]])<=range[k])
                                      {
                                          range[k]=D(n[k],n[preced[k]]);
                                          
                                          fprintf(fp2,"\n -----range[%d]= %f",k+1,range[k]);
                                      }
                                      
                                  }
                              }
                              
                          }
                          
                      }
        for(i=0;i<mxNo;i++)
        {
            if(i==min)
            {
                fprintf(fp4,"\nBefore Adaptation\nNode = %d Lifetime= %.2f",i+1,lifetime[i]);
                for(k=0;k<mxNo;k++)
                    
                {
                    if(Na[i][k]!=0)
                    {fprintf(fp4,"\nNeighbors=");
                        fprintf(fp4," %d",Na[i][k]+1);
                        
                    }
                }
            }
        }
        for(i=0;i<mxNo;i++)
        {
            
            for(k=0;k<mxNo;k++)
            {
                if(i!=k)
                {
                    dpn[i]=1;
                    
                    if(D(n[i],n[k])<=range[k])
                    {
                        cost[i][k]=1.0;
                        
                        Na[i][k]=k;
                        
                        Nn[i]++;
                    }
                    else if(D(n[i],n[k])>range[k])
                    {
                        cost[i][k]=INFINITE;
                        Na[i][k]=0;
                        
                    }
                    
                }
											     
            }
            
            
        }
        
        fprintf(fp1,"\n\n");
        fprintf(fp4,"\n\n");
        
        for(i=0;i<mxNo;i++)
        {
            for(k=0;k<mxNo;k++)
                fprintf(fp1,"\ncost[%d][%d]=%f",i+1,k+1,cost[i][k]);
            
        }
        
					   preced[mxNo];
					   memset ( preced, 0 , sizeof (preced));
        float distance[mxNo];
        shortpath(cost,preced,distance);
        
        for(i=0;i<mxNo;i++)
        {
            printf("\nNode %d, Predecessor= %d, Total cost= %f ",i+1,preced[i]+1,distance[i]);
            fprintf(fp3, "\nNode %d, Predecessor= %d, Total cost= %f ",i+1,preced[i]+1,distance[i]);
            total_cost=total_cost+distance[i];//sum of all cost
            
        }
        avg_cost=total_cost/mxNo;
        fprintf(fp4,"\ntotal cost= %f",total_cost);
        fprintf(fp4,"\navg cost= %f",avg_cost);
        
        
        
        printf("\n\n");
        fprintf(fp,"\n\n");
        
        for(i=0;i<mxNo;i++)
        {
            k=i;
            for(j=0;j<mxNo;j++)
            {
                p=preced[k];
                
                dpn[p]++;
                if(p==0)	break;
                k=p;
                
            }
        }
        
        for(i=0;i<mxNo;i++)
        {
            for(k=0;k<mxNo;k++)
            {
                if(i!=k)
                {
                    if(D(n[i],n[k])<=range[k])
                    {
                        o_dpn[i]=o_dpn[i]+dpn[k];
                        
                    }
                    
                }
            }
            
            To_dp=To_dp+o_dpn[i];
            
        }
        
        
        for(p=0;p<mxNo;p++)
        {
            
            I[p]=(((Irt*Trt)/Trui)+(((dpn[p])*Idt*Tdt)/Td)+(Nn[p]*(((Irr*Trr)/Trui)+(o_dpn[p]*(Idr*Tdr)/Td)))+((Is*Ts)/Td)+(8*Ip*Tp));
            
            lifetime[p]=cap[p]/(I[p]*10);
            fprintf(fp,"\nnode %d I= %f capacity= %f lifetime= %f",p+1,I[p],cap[p],lifetime[p]);
            total_I=total_I+I[p];
        }
        
        
        Lmax = lifetime[0];
        Lmin = lifetime[0];
        
        for(i=0;i<mxNo;i++)
        {
            if(Lmax<lifetime[i])
            {
                Lmax=lifetime[i];
                max=i;
            }
            if(Lmin>lifetime[i])
            {
                Lmin=lifetime[i];
                min=i;
            }
        }
        
        printf("\nLmax = %f ",Lmax);
        
        printf("\nLmin = %f",Lmin);
        fprintf(fp,"\nLmin = %f node= %d\n",Lmin,min+1);
        fprintf(fp4,"\nLmin = %f node= %d",Lmin,min+1);
        
        
        
        
        for(i=0;i<mxNo;i++)
        {
            if(i==w)
            {fprintf(fp4,"\nAfter Adaptation\nNode = %d Lifetime= %.2f",i+1,lifetime[i]);
							    	    
                for(k=0;k<mxNo;k++)
                    
                {
                    if(Na[i][k]!=0)
                    {fprintf(fp4,"\nNeighbors=");
                        fprintf(fp4," %d",Na[i][k]+1);
                        
                    }
                }
            }
        }
        
        
        avg_dp_ovrhrd=To_dp/mxNo;
        fprintf(fp,"\nAvg data packets overheard %.2f",avg_dp_ovrhrd);
        
        
        
    }
    
    //Picking up the node with Minimum Lifetime and Running Cost Range Adaptation Scheme for (Number of loops entered by the user/2), this will further enhance overall network lifetime.
    for(z=l/2;z<l;z++)
    {
        fprintf(fp1,"\n\n");
        fprintf(fp2,"\n\n");
        fprintf(fp4,"\n\n");
        fprintf(fp3,"\n\n");
        
        int w,j=2;
        fprintf(fp4,"\nCost adaptation loops");
        fprintf(fp4,"\nLoop No= %d",z+1);
        fprintf(fp3,"\nLoop No= %d",z+1);
        
        total_cost=0;
        To_dp=0;
	       o_dpn[mxNo];
	       memset (o_dpn,0,sizeof(o_dpn));
        Nn[mxNo];
	       memset (Nn,0,sizeof(Nn));
	       
        for(i=0;i<mxNo;i++)
        {
            if(i==min)
            {
                w=i;
                fprintf(fp4,"\nBefore Adaptation\nNode = %d Lifetime= %.2f",i+1,lifetime[i]);
                fprintf(fp2,"\nBefore Adaptation\nNode= %d",i+1);
                for(k=0;k<mxNo;k++)
                    
                {
                    if(Na[i][k]!=0)
                    {
                        fprintf(fp2,"\nNeighbors=");
                        fprintf(fp2," %d Load= %f",Na[i][k]+1,dpn[k]);
                        
                    }
                }
            }
            
        }
				 	  
        for(i=0;i<mxNo;i++)
        {
            
            for(k=0;k<mxNo;k++)
            {
                if(i!=k)
                {
                    dpn[i]=1;
                    
                    if(cost[i][k]==INFINITE)
                    {
                        Na[i][k]=0;
                        
                    }
                    else if(cost[i][k]!=INFINITE)
                    {
                        Nn[i]++;
                        Na[i][k]=k;
																								
                        if(i==min)//((Lmax+Lmin)/8)
                        {
                            
                            if(Na[i][k]!=0)
                            {
                                
                                cost[k][preced[k]]=j;
                                cost[preced[k]][k]=cost[k][preced[k]];
                                
                                
                            }
                        }
                        
                        if(cost[i][k]!=j)
                        {
                            cost[i][k] = 1;
                        }
                    }
                    
                }
                
            }
            
            
        }
        
        fprintf(fp1,"\n\n");
        fprintf(fp4,"\n\n");
        
        
        
					   preced[mxNo];
					   memset ( preced, 0 , sizeof (preced));
        float distance[mxNo];
        shortpath(cost,preced,distance);
        
        for(i=0;i<mxNo;i++)
        {
            printf("\nNode %d, Predecessor= %d, Total cost= %f ",i+1,preced[i]+1,distance[i]);
            fprintf(fp3, "\nNode %d, Predecessor= %d, Total cost= %f ",i+1,preced[i]+1,distance[i]);
            total_cost=total_cost+distance[i];//sum of all cost
            
        }
        avg_cost=total_cost/mxNo;
        
        
        printf("\n\n");
        fprintf(fp,"\n\n");
        
        for(i=0;i<mxNo;i++)
        {
            
            k=i;
            for(j=0;j<mxNo;j++)
            {
                p=preced[k];
                
                dpn[p]++;
                if(p==0)	break;
                k=p;
                
            }
        }
        
        for(i=0;i<mxNo;i++)
        {
            for(k=0;k<mxNo;k++)
            {
                if(i!=k)
                {
                    if(D(n[i],n[k])<=range[k])
                    {
                        o_dpn[i]=o_dpn[i]+dpn[k];
                        
                    }
                    
                }
            }
            
            To_dp=To_dp+o_dpn[i];
            
        }
        
        
        for(p=0;p<mxNo;p++)
        {
            fprintf(fp1," \nNode %d Tr.data packets(load)= %f",p+1,dpn[p]);
            
            I[p]=(((Irt*Trt)/Trui)+(((dpn[p])*Idt*Tdt)/Td)+(Nn[p]*(((Irr*Trr)/Trui)+(o_dpn[p]*(Idr*Tdr)/Td)))+((Is*Ts)/Td)+(8*Ip*Tp));
            
            lifetime[p]=cap[p]/(I[p]*10);
            fprintf(fp,"\nnode %d I= %f capacity= %f lifetime= %f",p+1,I[p],cap[p],lifetime[p]);
            total_I=total_I+I[p];
        }
        
        
        Lmax = lifetime[0];
        Lmin = lifetime[0];
        
        
        for(i=0;i<mxNo;i++)
        {
            
            if(Lmax<lifetime[i])
            {
                Lmax=lifetime[i];
                max=i;
            }
            if(Lmin>lifetime[i])
            {
                Lmin=lifetime[i];
                min=i;
            }
        }
        
        printf("\nLmax = %f ",Lmax);
					   
        printf("\nLmin = %f",Lmin);
        fprintf(fp,"\nLmin = %f node= %d\n",Lmin,min+1);
        
        
        for(i=0;i<mxNo;i++)
        {
            if(i==w)
            {
                fprintf(fp4,"\nAfter Adaptation\nNode = %d Lifetime= %.2f",i+1,lifetime[i]);
                fprintf(fp2,"\nAfter Adaptation\nNode = %d",i+1);
                for(k=0;k<mxNo;k++)
                    
                {
                    
                    if(Na[i][k]!=0)
                    {fprintf(fp2,"\nNeighbors=");
                        fprintf(fp2," %d Load= %f",Na[i][k]+1,dpn[k]);
                        
                    }
                }
            }
        }
        
        
        avg_dp_ovrhrd=To_dp/mxNo;
        fprintf(fp,"\nAvg data packets overheard %.2f",avg_dp_ovrhrd); 
        fprintf(fp,"\navg cost= %f",avg_cost); 
        
        
    }
    
    fclose(fp);
    fclose(fp1);
    fclose(fp2);
    fclose(fp3);
    fclose(fp4);
    return 0;
    
}



