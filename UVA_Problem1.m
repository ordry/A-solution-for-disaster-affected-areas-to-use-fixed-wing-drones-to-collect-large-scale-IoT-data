%clc;clear;warning off;
N=1024;
sidelength = 5000;
middle_point=0;
%x=round(rand(N,1)*sidelength);  %横坐标在(0，sidelength)范围内，生成1*N的随机矩阵
%y=round(rand(1,N)*sidelength);  %纵坐标在(0，sidelength)范围内，生成1*N的随机矩阵
%绘图
hold on
plot(x,y,'.')  %以x2为横坐标,y2为纵坐标绘制蓝色圆圈点图
rectangle('position',[0 0 sidelength sidelength],'LineWidth',2,'LineStyle','--');  %中心区域的虚线框
set(gca,'XLim',[0 sidelength]);       %X轴的数据显示范围
set(gca,'XTick',0:1000:sidelength); %设置要显示的坐标刻度
set(gca,'YLim',[0 sidelength]);       %Y轴的数据显示范围
set(gca,'YTick',0:1000:sidelength); %设置要显示的坐标刻度
xlabel('x','FontSize',15);ylabel('y','FontSize',15)

xCurrent=0; %当下位置x (0,0)为起点位置
yCurrent=0; %当下位置y
xFinal=sidelength; %终点位置(sidelength,sidelength)
yFinal=sidelength;
%采集前往的点的位置
xSearched=zeros(1,N);
ySearched=zeros(1,N);
series=1;
QsumFinal=0; %最后收集到的数据总量
E=360000; %总能量，单位ws
WiFi_Scope=400; %无人机无线网络通信半径
K=32; %信道数
Pfly=100; %直线飞行功率，每秒消耗100w，续航时长为60min
Psearch=100*2.31; %匀速圆周运动飞行功率 半径为30m
a=11.95;
b=0.136;

%运动信息
vsteady=30;
vcircle=12.91;
acceleration=5;
deleteMartix=zeros(1,N);
Rcycle=30; %无人机匀速圆周运动半径
rScope=WiFi_Scope+Rcycle; %数据接收范围

%初始最大值的设置
effienceMax=0;
EflyDecide=0;%决定要去的点的能量消耗
max_c=0;
maxi=1;
max_collection=zeros(1,N);

rate(1)=1; rate(2)=10; rate(3)=1;
rateSum=sum(rate);
rate(1)=rate(1)/rateSum; rate(2)=rate(2)/rateSum; rate(3)=rate(3)/rateSum;
Q=0.1*[ones(1,round(rate(1)*N)),zeros(1,N-round(rate(1)*N))];
Q=Q+12*[zeros(1,round(rate(1)*N)),ones(1,round(rate(2)*N)),zeros(1,N-round(rate(1)*N)-round(rate(2)*N))];
Q=Q+1*[zeros(1,round(rate(1)*N)+round(rate(2)*N)),ones(1,N-round(rate(1)*N)-round(rate(2)*N))];

dToFinal=distance1(xFinal,yFinal,xCurrent,yCurrent); %计算从起点到终点的飞行距离
tToFinal=(dToFinal-(vsteady^2)/acceleration)/vsteady;%时间
Eflya=2925.9; %0直线加速到30m/s的能耗
EflyToFinal=Pfly*tToFinal+2*Eflya;

while(E>=EflyToFinal)
    n=nnz(x);
    for i = 1 : n
        collection=zeros(1,N);
        c=0;
        Qsum=zeros(1,N);
        for j = 1 : n %计算i位置能收集到的数据总量
            if(distance1(x(i),y(i),x(j),y(j))<=rScope)
                c=c+1;
                collection(c)=j;
                Qsum(i)=Qsum(i)+Q(j);
            end
        end
        %计算时间

        d=distance1(xCurrent,yCurrent,x(i),y(i)); %计算从当下位置到i点的匀速飞行时间
        tFly=(d-(vsteady^2-vcircle^2)/acceleration)/vsteady;
        if(xCurrent==0&&yCurrent==0) %特判从起点到第一个点的飞行时间
            tFly=(d-(vsteady^2-vcircle^2)/(2*acceleration)-(vsteady^2)/(2*acceleration))/vsteady;
        end
        %计算从i点到终点的飞行时间
        dToFinal=distance1(xFinal,yFinal,x(i),y(i)); 
        tToFinal=(dToFinal-(vsteady^2-vcircle^2)/(2*acceleration)-(vsteady^2)/(2*acceleration))/vsteady;
        gamma=0.5; %衰减参数
        epsilonLOS=0; %扰动
        epsilonNLOS=0; %扰动
        tn=zeros(1,c);
        for j = 1 : c %计算每个设备所需收集时间
            theta=asin(100/distance1(x(i),y(i),x(collection(j)),y(collection(j))));

            RLOS=128*log(1+1000/(10000+distance1(x(i),y(i),x(collection(j)),y(collection(j)))^2))/log(2); %LOS传输速度
            epsilonLOS=randn(1,1)*(RLOS^2/36);
            RLOS=RLOS-epsilonLOS;

            RNLOS=128*log(1+gamma*1000/(10000+distance1(x(i),y(i),x(collection(j)),y(collection(j)))^2))/log(2); %NLOS传输速度
            epsilonNLOS=randn(1,1)*(RNLOS^2/36);
            RNLOS=RNLOS-epsilonNLOS;

            PLOS=1/(1+a*exp(-b*(theta-a)));
            tn(j)=Q(collection(j))/(PLOS*RLOS+(1-PLOS)*RNLOS);
        end
        %贪心算法计算K个信道下c个设备的收集时间
        tCom=0;
        if(c<=K) %计算i点位的搜集所需总速度
            tCom=max(tn);
        else
            tn=sort(tn,'descend');
            tTransmission=zeros(1,K);
            next=1; %K之后待分配的任务编号
            for m = 1 : K %选择传输时间最大的前K个数据
                tTransmission(m)=tn(m);
            end
            tmin=tn(K); %K个并行任务中第一个完成所需时间
            tCom=tCom+tmin;
            while(K+next<=c)
                for m = 1 : K
                    tTransmission(m)=tTransmission(m)-tmin; %K个并行任务中第一个完成后其余的剩余时间
                end
                for m = 1 : K
                    if(tTransmission(m)==0 && K+next<=c)
                        tTransmission(m)=tn(K+next); %对已完成任务的信道再分配任务
                        next=next+1;
                    end
                    if(K+next>c)
                        break;
                    end
                end
                if(K+next>c)
                    break;
                end
                tTransmission=sort(tTransmission,'descend');
                tmin=tTransmission(K); %再分配后K个并行任务中第一个完成所需时间
                tCom=tCom+tmin;
            end
            tCom=tCom+max(tTransmission);
        end

        Eflya=1040.6; %Eflya为加速能耗,Eflyai为减速能耗
        Eflyai=Eflya;
        if(xCurrent==0&&yCurrent==0) %特判从起点到第一个点的飞行能耗
            Eflya=2925.9;
        end
        Efly=Eflya+Pfly*tFly+Eflyai+Psearch*tCom; %加速能耗+匀速能耗+减速能耗+收集能耗（旋转能耗）

        %这里用作计算i点到终点的能耗
        Eflya=1040.6;
        Eflyai=2925.9;
        EflyToFinal=Eflya+Pfly*tToFinal+Eflyai; %加速能耗+匀速能耗+减速能耗
        %动态规划决策下一个前往的点
        if(E-Efly>=EflyToFinal)  %判断E-Efly>=E终-E降落
            if((effienceMax<Qsum(i)/Efly) || (effienceMax==Qsum(i)/Efly && Qmax<Qsum(i)))
                effienceMax=Qsum(i)/Efly;
                max_c=c;
                maxi=i;
                max_collection=collection;
                EflyDecide=Efly;
                Qmax=Qsum(i);
            end
        end
    end
    if(EflyDecide==0 || E-EflyDecide<EflyToFinal)
        break;
    end
    E=E-EflyDecide; %剩余能量计算
    xSearched(series)=x(maxi); %记录前往的点的位置
    ySearched(series)=y(maxi);
    series=series+1;
    xCurrent=x(maxi); %记录当下位置的改变
    yCurrent=y(maxi);
    QsumFinal=QsumFinal+Qmax; %总数据的变化
    for idelete = 1 : max_c %删除采集过的点
        deleteMartix(max_collection(idelete))=1;
    end
    x(deleteMartix==1)=[];
    y(deleteMartix==1)=[];
    Q(deleteMartix==1)=[];
    x(N,1)=0;
    y(1,N)=0;
    Q(1,N)=0;
    Qmax=0; %最值的初始化处理
    effienceMax=0;
    EflyDecide=0;
    max_c=0;
    maxi=1;
    max_collection=zeros(1,N);
    deleteMartix=zeros(1,N);
    middle_point=middle_point+1;
end
E=E-EflyToFinal;
xSearched(series)=xFinal; %前往终点
ySearched(series)=yFinal;
xpoint=[0,0];
ypoint=[xSearched(1),ySearched(1)];
drawArrow(xpoint,ypoint);
for i = 1 : series-1
    xpoint=[xSearched(i),ySearched(i)];
    ypoint=[xSearched(i+1),ySearched(i+1)];
    drawArrow(xpoint,ypoint);
end







