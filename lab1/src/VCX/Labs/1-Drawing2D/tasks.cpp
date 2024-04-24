#include <random>
#include <cmath>
#include <spdlog/spdlog.h>

#include "Labs/1-Drawing2D/tasks.h"

using VCX::Labs::Common::ImageRGB;



namespace VCX::Labs::Drawing2D {
    /******************* 1.Image Dithering *****************/
    void DitheringThreshold(
        ImageRGB &       output,
        ImageRGB const & input) {
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0,
                };
            }
    }

    void DitheringRandomUniform(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        std::uniform_real_distribution<double> distribution_real(-0.5,0.5);
        std::default_random_engine e; 
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                double k=distribution_real(e);
                color.r+=k;
                color.g+=k;
                color.b+=k;
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0
                };
            }
    }

    void DitheringRandomBlueNoise(
        ImageRGB &       output,
        ImageRGB const & input,
        ImageRGB const & noise) {
        // your code here:
        std::uniform_real_distribution<double> distribution_real(-0.5,0.5);
        std::default_random_engine e; 
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                double k=noise.At(x,y)[0]-0.5;
                //printf("%d %d",input.GetSizeX(),input.GetSizeY());
                color.r+=k;
                color.g+=k;
                color.b+=k;
                output.At(x, y) = {
                    color.r > 0.5 ? 1 : 0,
                    color.g > 0.5 ? 1 : 0,
                    color.b > 0.5 ? 1 : 0
                };
            }
    }

    void DitheringOrdered(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        double an[9][3][3];
        int matrix[9][2];
        matrix[0][0]=1;matrix[0][1]=1;
        matrix[1][0]=0;matrix[1][1]=1;
        matrix[2][0]=1;matrix[2][1]=2;
        matrix[3][0]=2;matrix[3][1]=1;
        matrix[4][0]=2;matrix[4][1]=0;
        matrix[5][0]=0;matrix[5][1]=2;
        matrix[6][0]=0;matrix[6][1]=0;
        matrix[7][0]=2;matrix[7][1]=2;
        matrix[8][0]=1;matrix[8][1]=0;
        for(int i=0;i<9;++i)
            for(int j=0;j<3;++j)
                for(int k=0;k<3;++k)
                    an[i][j][k]=0.0;
        for(int i=0;i<9;++i){
            for(int j=0;j<=i;++j)
                an[i][matrix[j][0]][matrix[j][1]]=1;
        }             
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {
                glm::vec3 color = input.At(x, y);
                printf("%lf\n",color.r);
                int p= color.r*9;
                for (std::size_t i = 0;i<3;i++)
                    for(std::size_t j = 0;j<3;j++)
                        output.At(3*x+i, 3*y+j) = {
                            an[p][j][i],
                            an[p][j][i],
                            an[p][j][i]
                        };    
            }
    }

    void DitheringErrorDiffuse(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        double d=0;
        double an[300][300]={0};
       
        for (std::size_t y = 0; y < input.GetSizeY(); ++y)
            for (std::size_t x = 0; x < input.GetSizeX(); ++x) {     
                glm::vec3 color = input.At(x, y);
                double p=color.r+an[x][y];
                d=p-(p >= 0.5 ? 1 : 0);
                double t=(p >= 0.5 ? 1 : 0);
                output.At(x, y) = {t,t,t};
                an[x+1][y]+=d*7.0/16.0;
                an[x+1][y+1]+=d/16.0;
                an[x][y+1]+=d*5.0/16.0;
                if(x-1>=0)
                    an[x-1][y+1]+=d*3.0/16.0;
            }
        
    }

    /******************* 2.Image Filtering *****************/
    void Blur(
        ImageRGB &       output,
        ImageRGB const & input) {
        // your code here:
        double an[3][3]={1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
        double sum=0.0;
        glm::vec3 color={0.0,0.0,0.0};
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {   
                color={0.0,0.0,0.0};
                sum=0.0;
                for(int i=-1;i<2;i++)
                    for(int j=-1;j<2;j++)
                        if(x+i>=0 && x+i<input.GetSizeX() && y+j>=0 && y+j<input.GetSizeY()){
                            color.r+=input.At(x+i,y+j)[0]*an[i+1][j+1];
                            color.g+=input.At(x+i,y+j)[1]*an[i+1][j+1];
                            color.b+=input.At(x+i,y+j)[2]*an[i+1][j+1];
                            sum+=an[i+1][j+1];
                        }
                output.At(x,y)={
                    color.r/sum,
                    color.g/sum,
                    color.b/sum
                };
            }
        
    }

    void Edge(
        ImageRGB &       output,
        ImageRGB const & input) {
        
        double an1[3][3]={
            1,2,1,
            0,0,0,
            -1,-2,-1};
        double an2[3][3]={
            -1,0,1,
            -2,0,2,
            -1,0,1
        };
        double sum1=0.0,sum2=0.0;
        glm::vec3 color1={0.0,0.0,0.0};
        glm::vec3 color2={0.0,0.0,0.0};
        for (std::size_t x = 0; x < input.GetSizeX(); ++x)
            for (std::size_t y = 0; y < input.GetSizeY(); ++y) {   
                color1={0.0,0.0,0.0};
                color2={0.0,0.0,0.0};
                for(int i=-1;i<2;i++)
                    for(int j=-1;j<2;j++){
                        if(i==0&&j==0)continue;
                        if(x+i>=0 && x+i<input.GetSizeX() && y+j>=0 && y+j<input.GetSizeY()){
                            color1.r+=input.At(x+i,y+j)[0]*an1[i+1][j+1];
                            color1.g+=input.At(x+i,y+j)[1]*an1[i+1][j+1];
                            color1.b+=input.At(x+i,y+j)[2]*an1[i+1][j+1];
                            color2.r+=input.At(x+i,y+j)[0]*an2[i+1][j+1];
                            color2.g+=input.At(x+i,y+j)[1]*an2[i+1][j+1];
                            color2.b+=input.At(x+i,y+j)[2]*an2[i+1][j+1];
                            //sum1+=an1[i+1][j+1];
                            //printf("%zd %zd %lf\n",x,y,sum);
                        }
                        
                    }
                    /*color.r-=input.At(x,y)[0]*sum;
                color.g-=input.At(x,y)[1]*sum;
                color.b-=input.At(x,y)[2]*sum;*/
                
                output.At(x,y)={
                    sqrt(color1.r*color1.r+color2.r*color2.r),//>0? color.r:-color.r,
                    sqrt(color1.g*color1.g+color2.g*color2.g),//>0? color.g:-color.g,
                    sqrt(color1.b*color1.b+color2.b*color2.b)//>0? color.b:-color.b
                };
            }
    }

    /******************* 3. Image Inpainting *****************/
    void Inpainting(
        ImageRGB &         output,
        ImageRGB const &   inputBack,
        ImageRGB const &   inputFront,
        const glm::ivec2 & offset) {
        output             = inputBack;
        std::size_t width  = inputFront.GetSizeX();
        std::size_t height = inputFront.GetSizeY();
        glm::vec3 * g      = new glm::vec3[width * height];
        memset(g, 0, sizeof(glm::vec3) * width * height);
        // set boundary condition
        for (std::size_t y = 0; y < height; ++y) {
            // set boundary for (0, y), your code: g[y * width] = ?
            // set boundary for (width - 1, y), your code: g[y * width + width - 1] = ?
            g[y*width]=inputBack.At(offset[0],offset[1]+y)-inputFront.At(0, y);
            g[y*width+width-1]=inputBack.At(offset[0]+width-1,offset[1]+y)-inputFront.At(width-1, y);
        }
        for (std::size_t x = 0; x < width; ++x) {
            // set boundary for (x, 0), your code: g[x] = ?
            // set boundary for (x, height - 1), your code: g[(height - 1) * width + x] = ?
            g[x]=inputBack.At(offset[0]+x,offset[1])-inputFront.At(x, 0);
            g[(height - 1) * width + x] = inputBack.At(offset[0]+x,offset[1]+height-1)-inputFront.At(x, height - 1);
        }

        // Jacobi iteration, solve Ag = b
        for (int iter = 0; iter < 8000; ++iter) {
            for (std::size_t y = 1; y < height - 1; ++y)
                for (std::size_t x = 1; x < width - 1; ++x) {
                    g[y * width + x] = (g[(y - 1) * width + x] + g[(y + 1) * width + x] + g[y * width + x - 1] + g[y * width + x + 1]);
                    g[y * width + x] = g[y * width + x] * glm::vec3(0.25);
                }
        }

        for (std::size_t y = 0; y < inputFront.GetSizeY(); ++y)
            for (std::size_t x = 0; x < inputFront.GetSizeX(); ++x) {
                glm::vec3 color = g[y * width + x] + inputFront.At(x, y);
                output.At(x + offset.x, y + offset.y) = color;
            }
        delete[] g;
    }

    /******************* 4. Line Drawing *****************/
    void DrawLine(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1) {
        // your code here:
        int x0=p0[0],y0=p0[1],x1=p1[0],y1=p1[1];
        if(x1<x0){
            int t=x1;
            x1=x0;
            x0=t;
            t=y1;
            y1=y0;
            y0=t;}
        int ry0=2*y0;
        int flag=0;
        if(y1<y0){
            flag=1;
            y1=ry0-y1;
        }
        if(y1-y0<=x1-x0){
            int x , y = y0;
            int dx = 2*( x1-x0 ) , dy = 2*( y1-y0 );
            int dydx = dy-dx , F = dy-dx /2;
            if(flag==0){
                for ( x=x0 ; x<=x1 ; x++) {
                    canvas.At(x,y)=color;
                    if (F<0) F += dy;
                    else {
                        y++; 
                        F += dydx; 
                    }
                }
            }
            else{
                for ( x=x0 ; x<=x1 ; x++) {
                    canvas.At(x,ry0-y)=color;
                    if (F<0) F += dy;
                    else {
                        y++; 
                        F += dydx; 
                    }
                }
            }         
        }
        else{
            int x=x0 , y;
            int dx = 2*( x1-x0 ) , dy = 2*( y1-y0 );
            int dxdy = dx-dy , F = dx-dy /2;
            if(flag==0){
                for ( y=y0 ; y<=y1 ; y++) {
                    canvas.At(x,y)=color;
                    if (F<0) F += dx;
                    else {
                        x++; 
                        F += dxdy; 
                    }
                }
            }
            else{
                for ( y=y0 ; y<=y1 ; y++) {
                    canvas.At(x,ry0-y)=color;
                    if (F<0) F += dx;
                    else {
                        x++; 
                        F += dxdy; 
                    } 
                }
            }
        }
        

    }
        

    /******************* 5. Triangle Drawing *****************/
    void DrawTriangleFilled(
        ImageRGB &       canvas,
        glm::vec3 const  color,
        glm::ivec2 const p0,
        glm::ivec2 const p1,
        glm::ivec2 const p2) {
        // your code here:
        glm::ivec2 top=p0,mid=p1,btm=p2,min;
        if(top[1]<mid[1]){
		    min=mid;
            mid=top;
            top=min;}
	    if(top[1]<btm[1]){
		    min=btm;
            btm=top;
            top=min;}
        if(btm[1]>mid[1]){
            min=btm;
            btm=mid;
            mid=min;}
        float x1=top[0],y1=top[1],x2=mid[0],y2=mid[1],x0=btm[0],y0=btm[1];
        if(y2==y1){
            double grad01=(float)(x1-x0)/(float)(y1-y0);
            double grad02=(float)(x2-x0)/(float)(y2-y0);
            if(x2*(y1-y0)>=(x1-x0)*(y2-y0)+x0*(y1-y0)){
            //中间点在右侧
                double left=(double)(x0),right=(double)(x0);            
                for (int y=y0;y<=y2;y++){
                    for(int x=(int)left;x<=(int)right;x++)canvas.At(x,y)=color;
                    left+=grad01;
                    right+=grad02;
                }
            }
            else{
                double left=(double)(x0),right=(double)(x0);          
                for (int y=y0;y<=y2;y++){
                    for(int x=(int)left;x<=(int)right;x++)canvas.At(x,y)=color;
                    left+=grad02;
                    right+=grad01;
                }
            }
        }
        else if(y2==y0){
            double grad01=(float)(x1-x0)/(float)(y1-y0);
            double grad21=(float)(x1-x2)/(float)(y1-y2);
            if(x2*(y1-y0)>=(x1-x0)*(y2-y0)+x0*(y1-y0)){
            //中间点在右侧
                double left=(double)(x0),right=(double)(x0);            
                for (int y=y0;y<=y1;y++){
                    for(int x=(int)left;x<=(int)right;x++)canvas.At(x,y)=color;
                    left+=grad01;
                    right+=grad21;
                }
            }
            else{
                double left=(double)(x0),right=(double)(x0);          
                for (int y=y0;y<=y1;y++){
                    for(int x=(int)left;x<=(int)right;x++)canvas.At(x,y)=color;
                    left+=grad21;
                    right+=grad01;
                }
            }
        }
        else{
            double grad01=(float)(x1-x0)/(float)(y1-y0);
        double grad02=(float)(x2-x0)/(float)(y2-y0);
        double grad21=(float)(x1-x2)/(float)(y1-y2);
        if(x2*(y1-y0)>=(x1-x0)*(y2-y0)+x0*(y1-y0)){
            //中间点在右侧
            double left=(double)(x0),right=(double)(x0);            
            for (int y=y0;y<y2;y++){
                for(int x=(int)left;x<=(int)right;x++)canvas.At(x,y)=color;
                left+=grad01;
                right+=grad02;
            }
            for (int y=y2;y<=y1;y++){
                for(int x=(int)left;x<=(int)right;x++)canvas.At(x,y)=color;
                left+=grad01;
                right+=grad21;
            }
        }
        else{
            double left=(double)(x0),right=(double)(x0);          
            for (int y=y0;y<y2;y++){
                for(int x=(int)left;x<=(int)right;x++)canvas.At(x,y)=color;
                left+=grad02;
                right+=grad01;
            }
            for (int y=y2;y<=y1;y++){
                for(int x=(int)left;x<=(int)right;x++)canvas.At(x,y)=color;
                left+=grad21;
                right+=grad01;
            }
        }
        }
    }

    /******************* 6. Image Supersampling *****************/
    void Supersample(
        ImageRGB &       output,
        ImageRGB const & input,
        int              rate) {
        // your code here:
        
        int output_w=output.GetSizeX();
        int output_l=output.GetSizeY();
        int input_w=input.GetSizeX();
        int input_l=input.GetSizeY();
        double comput_rate=(double)input_w/(double)(output_w*rate);
        ImageRGB midput(output_w*rate,output_l*rate);
        
        std::size_t t,s;
        double p,q;
        double m,n;
        int flag_w=0,flag_l=0;
        for (std::size_t x = 0; x < output_w*rate; ++x)
            for (std::size_t y = 0; y < output_l*rate; ++y) {
                m=((double)x+0.5)*comput_rate-0.5;
                n=((double)y+0.5)*comput_rate-0.5;
                t=(int)m;
                s=(int)n;
                p=m-(double)t;
                q=n-(double)s;
                if(m<=0.0){
                    if(n<=0.0)
                        midput.At(x,y)=input.At(0,0);
                    else if(n>=(double)input_l-1.0)
                        midput.At(x,y)=input.At(0,input_l-1);
                    else
                        midput.At(x,y)={
                            input.At(0,s)[0]*(1-q)+input.At(0,s+1)[0]*q,
                            input.At(0,s)[1]*(1-q)+input.At(0,s+1)[1]*q,
                            input.At(0,s)[2]*(1-q)+input.At(0,s+1)[2]*q};                    
                }
                else if(m>=(double)input_w-1.0){
                    if(n<=0.0)
                        midput.At(x,y)=input.At(input_w-1,0);
                    else if(n>=(double)input_l-1.0)
                        midput.At(x,y)=input.At(input_w-1,input_l-1);
                    else
                        midput.At(x,y)={
                            input.At(input_w-1,s)[0]*(1-q)+input.At(input_w-1,s+1)[0]*q,
                            input.At(input_w-1,s)[1]*(1-q)+input.At(input_w-1,s+1)[1]*q,
                            input.At(input_w-1,s)[2]*(1-q)+input.At(input_w-1,s+1)[2]*q};                    
                }
                else if(n<=0.0)
                    midput.At(x,y)={
                        input.At(t,0)[0]*(1-p)+input.At(t+1,0)[0]*p,
                        input.At(t,0)[1]*(1-p)+input.At(t+1,0)[1]*p,
                        input.At(t,0)[2]*(1-p)+input.At(t+1,0)[2]*p
                    };
                else if(n>=(double)input_l-1.0)
                    midput.At(x,y)={
                        input.At(t,input_l-1)[0]*(1-p)+input.At(t+1,input_l-1)[0]*p,
                        input.At(t,input_l-1)[1]*(1-p)+input.At(t+1,input_l-1)[1]*p,
                        input.At(t,input_l-1)[2]*(1-p)+input.At(t+1,input_l-1)[2]*p
                    };
                else
                    midput.At(x,y)={
                        (input.At(t,s)[0]*(1-p)+input.At(t+1,s)[0]*p)*(1-q)+(input.At(t,s+1)[0]*(1-p)+input.At(t+1,s+1)[0]*p)*q,
                        (input.At(t,s)[1]*(1-p)+input.At(t+1,s)[1]*p)*(1-q)+(input.At(t,s+1)[1]*(1-p)+input.At(t+1,s+1)[1]*p)*q,
                        (input.At(t,s)[2]*(1-p)+input.At(t+1,s)[2]*p)*(1-q)+(input.At(t,s+1)[2]*(1-p)+input.At(t+1,s+1)[2]*p)*q
                    };             
            }

        double r,g,b;
        std::size_t xi,yi;
        glm::vec3 color;
       
        for (std::size_t x = 0; x < output.GetSizeX(); ++x)
            for (std::size_t y = 0; y < output.GetSizeY(); ++y) {
                r=0.0;
                g=0.0;
                b=0.0;                
                for(int i=0;i<rate;i++)
                    for(int j=0;j<rate;j++){
                        xi=x*rate+i;
                        yi=y*rate+j;
                        color = midput.At(xi, yi);
                        r+=color.r;
                        g+=color.g;
                        b+=color.b;  
                    }
                r/=(double)(rate*rate);
                g/=(double)(rate*rate);
                b/=(double)(rate*rate);
                //printf("%lf %lf %lf %d %zd %lf %lf %lf, %zd %zd\n",r,g,b,rate,y,input.At(x,y)[0],input.At(x,y)[1],input.At(x,y)[2],input.GetSizeX(),input.GetSizeY());
                output.At(x,y)={r,g,b};
            }
        
        
    }

    /******************* 7. Bezier Curve *****************/
    // Note: Please finish the function [DrawLine] before trying this part.
    glm::vec2 CalculateBezierPoint(
        std::span<glm::vec2> points,
        float const          t) {
        // your code here:
        glm::vec2 sum={0.0,0.0};
        int n=points.size();
        for (int i=0;i<n;i++){
            double p=1.0;
            for (int j=0;j<i;j++){
                p*=n-1-j;
                p/=j+1;
                p*=t;
            }
            for(int j=0;j<n-1-i;j++){
                p*=1-t;
            }
            sum[0]+=p*points[i][0];
            sum[1]+=p*points[i][1];
            
        }
        return sum;
    }
} // namespace VCX::Labs::Drawing2D