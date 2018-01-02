%---------------MAIN FUNCTION-----------------------%
%图像预处理，整体思路：
%获得输入文件路径
%灰度直方图阈值分割
%区域扫描统计去干扰线和孤立噪点
%连通分量去掉干扰块
%滴水算法
clc;clear;close all;
flag=1;%方便调试而设立的flag=1为调试开启
low=0.09;up=0.18;%如果采用双阈值分割才会用到
bound=3;%区域扫描边界
ns=4;%字符个数改成5既可用于5个字符的
road='D:\matlab program\CAPTCHA\4nngnA1\';
name=textread('name3.txt','%s');
allsum=length(name);%文件总数
cnt=1;
if(flag==1)
    allsum=cnt;
end
while cnt<=allsum
    dirname=name{cnt};
    a=imread([road,dirname]);
    cnt=cnt+1
    if length(size(a))==3
        a_gray=rgb2gray(a);
    else
        a_gray=a;
    end
    [m,n]=size(a_gray);

%------根据灰度值出现次数过滤掉噪声-------%

    a_hist=imhist(a_gray);
    l=length(a_hist);
    a_cum=floor(([a_hist(2:l); 0]+[a_hist(3:l); 0 ;0]+a_hist+[0;0;a_hist(1:l-2)]+[0;a_hist(1:l-1)]));%3次平滑
    if (flag==1)
        figure(1);imshow(a_gray);
        figure(2);bar(a_cum);
    end
%----------在平滑图的基础上确定直方图谷值--%
    valley=zeros(1,10);
    is_filter=ones(1,256);
    valley(1)=1;
    k=2;
    for i=3:l-2
        if a_cum(i)<=8
        if a_cum(i)<=1&&((a_cum(i-1)>a_cum(i)&&a_cum(i-2)>=a_cum(i-1))||(a_cum(i+1)>a_cum(i)&&a_cum(i+2)>=a_cum(i+1)))
            valley(k)=i;
            k=k+1;
        end
        end
    end
    valley(k)=256;

% ---------单阈值分割--------------------%
bw=graythresh(a_gray);%采用最大化方法
a_bw=im2bw(a_gray,bw);
a_bw=im2bw(a_gray,0.6);%可调阈值
% ---------双阈值分割，枚举起点和终点--------------------%
%     for i=1:k-1
%         for j=i+1:k
%             percent=sum(a_hist(valley(i):valley(j)))/(m*n);
%             if percent>low&&percent<up
%                 for x=1:m
%                     for y=1:n
%                         if a_gray(x,y)>=valley(i)&&a_gray(x,y)<=valley(j)
%                             a_bw(x,y)=0;
%                         else
%                             a_bw(x,y)=1;
%                         end
%                     end
%                 end
%                 break;
%             end
%         end
%         if percent>low&&percent<up
%              break;
%         end
%     end
    
    if (flag==1)
        figure(3);imshow(a_bw);
    end
%-------------去除残余干扰线-----------------%
    a_bw2=[ones(1,n+2);ones(m,1) a_bw ones(m,1);ones(1,n+2)];%白色扩张
    for i=2:m+1
        for j=2:n+1
         if a_bw2(i,j)==0
             s=length(find(a_bw2(i-1:i+1,j-1:j+1)==0));
             if s<=bound    %若此3*3方格中0个数小于一定的阈值(需进一步调整)，将其颜色设为1
                  a_bw2(i,j)=1;
             end
         end
        end
    end
    a_bw=a_bw2(2:m+1,2:n+1);
    if (flag==1)
    figure(4);imshow(a_bw);
    end
%----------连通分量去区域------------------%
    a_bw=leach(a_bw);

%--------------按列投影--------------------%
    hor=m-sum(a_bw);
    ph_hor=pinghua(hor);

%--------------------------图像分割----------------------------%
    %计算可能起始位置
    %寻找左右边界
    left=1;
    right=n;
    for j=1:n
        if ph_hor(j)>0
            left=j;
            break;
        end
    end
    for j=n:-1:1
        if ph_hor(j)>0
            right=j;
            break;
        end
    end
%-------寻找切割低谷算法-----------------------%
    %-------方法一---------------------%
    %寻找极小点方法一用于粘连情况 
    %直方图截断
    %I=sort(ph_hor(left+1:right-1));
    % ct=0;
    % for i=1:n
    %     if I(i+1)>I(i)
    %     ct=ct+1;
    %     if ct>4
    %         ct=I(i+1);
    %         break;
    %     end
    %     end
    % end
    % ph_hor=ph_hor-ct;
    % ph_hor(find(ph_hor<0))=0;

    if (flag==1)
        figure(6);bar(ph_hor);
    end

    segp=zeros(ns-1,1);%所有切割起点的位置向量
    path=n*ones(m,ns-1);

    i=left+4;
    k=1;
    while i<right
        if ph_hor(i)<=0&&(ph_hor(i-1)>=1||ph_hor(i-2)>=1)
                init=ph_hor(i);
                if k<=ns-1
                    segp(k)=i;
                    k=k+1;
                end
                while i<right&&abs(ph_hor(i)-init)<=1
                    i=i+1;
                end
    %用于粘连情况效果不好                
    %     elseif ph_hor(i)>2&&ph_hor(i)<10&&(sum(ph_hor(i-3:i-1))>5*ph_hor(i))&&ph_hor(i)<ph_hor(i+1)
    %
    %         init=ph_hor(i);
    %             if k<=ns-1
    %                 segp(k)=i;
    %                 k=k+1;
    %             end
    %             while i<right&&abs(ph_hor(i)-init)<=1
    %                 i=i+1;
    %             end
        end
        i=i+1;
    end

    %-------------方法二等距估计算法----------------------%
    % for i=1:3
    %     bas=floor((right-left)*i/4+left);
    %     le=bas-floor(n/12);
    %     ri=bas+floor(n/12);
    %     f=find(hor(le:ri)==0);
    %     if f
    %         path(:,i)=(le+f(end)-1)*ones(m,1); %若直方图中有零点，可直接分割
    %          segp(i)=le+f(end)-1;
    %     else
    %         for j=le:ri
    %             ave1=sum(hor(j-2:j));
    %             ave2=sum(hor(j-1:j+1));
    %             if abs(ave1-ave2)>=10   %可调参数
    %                 segp(i)=j;%若无零点，则找到有突变的地方开始滴水算法
    %             end    
    %         end
    %     end
    %         
    % end
%执行滴水算法
lastx=0;
lasty=0;
deltax=[1;1;1;0;0];deltay=[0;1;-1;1;-1];%改变这两个向量后面两个的位置，将会使水滴在第一次遭遇平面时流向不同
for s=1:ns-1
    if segp(s)~=0
    j=segp(s);
    i=1;
    lastx=i;
    lasty=j;
    while(i<m)%当当前点达到图像底部时结束
        W=0;ww=zeros(5,1);
        for k=1:5
            ww(k)=a_bw(i+deltax(k),j+deltay(k))*(6-k);
        end
        W=max(ww);
        if W==0
            if rand<0.8
                W=5;
            else
                W=4;
            end
        end
        if W==1%避免回流
           if lasty<j
               W=2;
           end
        end
        if W==2%避免回流
           if lasty>j
               W=1;
           end
        end
        lastx=i;
        lasty=j;
        i=i+deltax(6-W);
        j=j+deltay(6-W);
        path(i,s)=min(path(i,s),j);
    end
    path(m,s)=path(m-1,s);
    end
end
%得到了ns-1条分割路径，分隔出单个验证码
bw=zeros(m,n,ns);

for i=1:ns
    bw(:,:,i)=a_bw;
end
for k=1:ns
    if k==1
        for i=1:m
         bw(i,path(i,k)+1:n,k)=1;
        end
    elseif k==ns
        for i=1:m
            bw(i,1:path(i,k-1),k)=1;
        end
    else
        for i=1:m
            bw(i,1:path(i,k-1),k)=1;
            bw(i,path(i,k)+1:n,k)=1;
        end
    end
end

if (flag==1)
    figure(7);
    for k=1:ns
        subplot(2,3,k);imshow(bw(:,:,k));
    end
end


    %---------去除空白打印图片---------------%
    if flag==1  
        figure(8);
    end
    for k=1:ns
        rowsum=sum(bw(:,:,k)');
        colsum=sum(bw(:,:,k));
        le=1;ri=n;up=1;do=m;%左右上下
        for i=1:m-1
            if rowsum(i)==n&&rowsum(i+1)<n
                up=i;
                break;
            end
        end
        for i=m:-1:2
            if rowsum(i)==n&&rowsum(i-1)<n
                do=i;
                break;
            end
        end
        for j=1:n-1
            if colsum(j)==m&&colsum(j+1)<m
                le=j;
                break;
            end
         end
        for j=n:-1:2
            if colsum(j)==m&&colsum(j-1)<m
                ri=j;
                break;
            end
        end
        img=bw(up+1:do-1,le+1:ri-1,k);
        img2=imresize(img,[16 16]);
        if flag==1  
             subplot(2,3,k);imshow(img2);
        end
        liwai=['c','i','j','k','o','p','s','u','v','w','x','z'];
        f=find(liwai==dirname(k));
        lenf=length(f);
        if lenf~=0&&dirname(k)>='a'&&dirname(k)<='z'
            writeroad=['D:\matlab program\samples\',upper(dirname(k)),'\'];
        elseif dirname(k)>='a'&&dirname(k)<='z'
            writeroad=['D:\matlab program\samples\',dirname(k),'1\'];
        else
            writeroad=['D:\matlab program\samples\',dirname(k),'\'];
        end
        hxx=[num2str((cnt-1)*ns+k+1000),'.jpg'];
        imwrite(img2,[writeroad,hxx]);
    end
end

%---------------SUBFUNCTIONS-----------------------%
function img=leach(bw)
%广度优先搜索去除噪点
[m, n]=size(bw);
area=0;queue=zeros(m*n,2);%构造了一个队列
seen=zeros(m,n);%避免重用
m_area=zeros(50,1);%连通分量面积
for i=1:m
    for j=1:n
        if bw(i,j)==0&&seen(i,j)==0
            area=area+1;
            queue(1,:)=[i,j];
            front=1;tail=2;%头指针和尾指针
            seen(i,j)=area;
            while front<tail
                x=queue(front,1);
                y=queue(front,2);
                seen(x,y)=area;
                m_area(area)= m_area(area)+1;
                front=front+1;
                if x+1<=m&&seen(x+1,y)==0&&bw(x+1,y)==0
                    queue(tail,:)=[x+1 ,y];
                    tail=tail+1;seen(x+1,y)=-1;
                end
                if x-1>=1&&seen(x-1,y)==0&&bw(x-1,y)==0
                    queue(tail,:)=[x-1 ,y];
                    tail=tail+1;seen(x-1,y)=-1;
                end
                if y+1<=n&&seen(x,y+1)==0&&bw(x,y+1)==0
                    queue(tail,:)=[x ,y+1];
                    tail=tail+1;seen(x,y+1)=-1;
                end
                if y-1>=1&&seen(x,y-1)==0&&bw(x,y-1)==0
                    queue(tail,:)=[x ,y-1];
                    tail=tail+1;seen(x,y-1)=-1;
                end
            end
        end
    end
end
s=find(m_area>0&m_area<=10);%面积参数可修改
img=bw;
for k=1:length(s)
    for i=1:m
    for j=1:n
    if(seen(i,j)==s(k))
        img(i,j)=1;
    end
    end
    end
end




function y=pinghua(s)
[m,n]=size(s);
if m==1
    s=s';
else
    n=m;
end
y=floor(([s(2:n);0]+s+[0;s(1:n-1)])/3);
% y=floor(([s(2:n);0]+[s(3:n); 0 ;0]+s+[0;0;s(1:n-2)]+[0;s(1:n-1)])/5);



