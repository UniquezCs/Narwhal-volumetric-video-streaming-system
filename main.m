clear;
Version=8;

time=10;%预测带宽时长(s)
FPS=30;
f=10;%每个gof有10帧
Frames=time*FPS;%总共300帧
G=Frames/f;%视频总共30个GOF
%变量
NC=[4,8];

%读取带宽

bw1 = xlsread('Static/B_2020.02.14_13.21.26.csv',1,'M301:M450');
bw1=bw1/1000;
%
bw2 = xlsread('Static/B_2020.02.13_13.57.29.csv',1,'M300:M451');
bw2=bw2/1000;

BW=zeros(2,G);

for i=1:G
    BW(1,i)=bw1(ceil(i/3));
     BW(2,i)=bw2(ceil(i/3));
    %BW(1,i)=bw1(i);
    %BW(2,i)=bw2(i);
end
av_bw1=sum(BW(1,:))/G;
av_bw2=sum(BW(2,:))/G;
load BW.mat
av_bw1=sum(BW(1,:))/G;
av_bw2=sum(BW(2,:))/G;
R=5;
S=4;
%-----------------------------------------------------------点数生成----------------------------------
Se_points=[780000,2880000];
point_1_1=zeros(S,G,1,R);
point_2_3=zeros(S,G,12,R);
point_smt=zeros(S,G,10,R);
point_3_4=zeros(S,G,36,R);
Tile=[1,12,36,12];

% for s=1:2
%     for ti=1:4
%         tmp=Se_points(s)/Tile(ti);
%         for k=1:Tile(ti)
%             tmp1=normrnd(tmp,tmp*0.03);
%             for g=1:G
%                 tmp=normrnd(tmp1,tmp1*0.01);
%                 for r=1:5
%                     if ti==1
%                         point_1_1(s,g,k,r)=round((tmp)*r^(1/1.7));
%                     elseif ti==2
%                         point_2_3(s,g,k,r)=round((tmp)*r^(1/1.7));
%                     elseif ti==3
%                         point_3_4(s,g,k,r)=round((tmp)*r^(1/1.7));
%                     else
%                         point_smt(s,g,k,r)=round((tmp)*r^(1/1.7));
%                     end
%                 end
%             end
%         end
%     end
% end
% % %-------------------------------------------------------------------------------------------------------------------------------------%
load points.mat
%------------------------------------------------读取视角信息--------------------------------------------------------------------------%%
cvpb=[1,0.8,0.66,0.57];
tile=[1,12,36,12];
cv1=zeros(G,1);
cv2=zeros(G,12);
cv3=zeros(G,36);
cv4=zeros(G,12);
for g=1:G
    for v=1:4
        if v==1
            for k=1:tile(v)
                if rand>1-cvpb(v)
                    cv1(g,k)=1;
                end
            end
        elseif v==2
            for k=1:tile(v)
                if rand>1-cvpb(v)
                    cv2(g,k)=1;
                end
            end
        elseif v==3
            for k=1:tile(v)
                if rand>1-cvpb(v)
                    cv3(g,k)=1;
                end
            end
        elseif v==4
            for k=1:tile(v)
                if rand>1-cvpb(v)
                    cv4(g,k)=1;
                end
            end
        end     
    end
end
load view.mat
%------------------------------------------------------------------------------------------------------------------------------------%
ratio=[7,400];
decode_ratio=[1,10];
for coder=1:2
for s=1:S
    for nc=1
        for bw=1:2
            for TileScheme=[1,4] %切块方案:1,12,36,语义切分
                if TileScheme==1
                    cv=cv1;
                    pointN=point_1_1;
                    K=size(pointN,3);%切块数目
                    GOF_K=ones(1,G)*size(pointN,3);%每个GOF的切块数目                    
                elseif TileScheme==2
                    cv=cv2;
                    pointN=point_2_3;
                    K=size(pointN,3);%切块数目
                    GOF_K=ones(1,G)*size(pointN,3);%每个GOF的切块数目
                elseif TileScheme==3
                    cv=cv3;
                    pointN=point_3_4;
                    K=size(pointN,3);%切块数目
                    GOF_K=ones(1,G)*size(pointN,3);%每个GOF的切块数目
                elseif TileScheme==4
                    cv=cv4;
                    pointN=point_smt;
                    GOF_K=ones(1,G)*size(pointN,3);%每个GOF的切块数目
                    K=12;%最大切块数目
                end
                
                plysize=zeros(G,K,R);%每个切块序列未压缩文件大小
                
                %ply文件大小
                
                plysize(:,:,:)=pointN(s,:,:,:)*17/1024/1024*8;
                %压缩文件大小
                binsize=plysize/ratio(coder);
                decodetimeTile=zeros(G,K,R);
                %对应的解码时间
                decodetimeTile(:,:,:)=roundn(0.0004356*decode_ratio(coder)*pointN(s,:,:,:)+45,-2)/1000;
                
                b0=10;%初始buffer大小,加载5个GoF后开始播放
                e3=zeros(G,K);            
                %----------------------------------------------------------------非自适应传输最低质量------------------------------------------------------------------------------------
                pausetime1=0;
                pausecount1=0;
                qoe1=0;
                b=b0*1/3;
                flag=1;%播放or缓冲
                starttime=1;
                transdata1=zeros(1,G);
                downloadtime1=zeros(1,G);
                for g=1:G
                    downloadtime1(g)=starttime;
                    [x,~,pausetime,pausecount,b,xr,flag,qoe,endtime]=TileSelect(cv,plysize,decodetimeTile,BW(bw,:),1,b0,NC(nc),b,g,binsize,GOF_K,flag,starttime);
                    starttime=endtime;
                    qoe1=qoe+qoe1;
                    pausetime1=pausetime+pausetime1;
                    pausecount1=pausecount+pausecount1;
                    transdata1(g)=sum(binsize(g,:,1).*cv(g,:));
                end
                
                decodetime1=sum(sum(reshape(decodetimeTile(:,:,1),G,K).*cv(:,:)))/NC(nc);
                xv1=1;
                %qoe1=A*xv1-B*pausetime1+D;
                %---------------------------------------------------------------非自适应传输最高质量-----------------------------------------------------------------------------------------
                pausetime5=0;
                pausecount5=0;
                qoe5=0;
                b=b0*1/3;
                flag=1;%播放or缓冲
                starttime=1;
                transdata5=zeros(1,G);
                for g=1:G
                    downloadtime5(g)=starttime;
                    [x,~,pausetime,pausecount,b,xr,flag,qoe,endtime]=TileSelect(cv,plysize,decodetimeTile,BW(bw,:),5,b0,NC(nc),b,g,binsize,GOF_K,flag,starttime);
                    starttime=endtime;
                    qoe5=qoe+qoe5;
                    pausetime5=pausetime+pausetime5;
                    pausecount5=pausecount+pausecount5;
                    transdata5(g)=sum(binsize(g,:,5).*cv(g,:));
                end
                
                decodetime5=sum(sum(reshape(decodetimeTile(:,:,5),G,K).*cv(:,:)))/NC(nc);
                xv5=5;
                %qoe5=A*xv5-B*pausetime5+D;
                %--------------------------------------------------------------------------------------------------------------------------------------------------------------
                
                %------------------------------------------------------传统自适应传输-------------------------------------------------------------------------------------------
                x2=zeros(1,G);
                xr2=zeros(G,K,R);
                pausetime2=0;
                pausecount2=0;
                qoe2=0;
                b=b0*1/3;
                flag=1;%播放or缓冲
                starttime=1;
                transdata=0;
                transdata2=zeros(1,G);
                for g=1:G
                    downloadtime2(g)=starttime;
                    [x,~,pausetime,pausecount,b,xr,flag,qoe,endtime,transdata]=TileSelect(cv,plysize,decodetimeTile,BW(bw,:),2,b0,NC(nc),b,g,binsize,GOF_K,flag,starttime);                                      
                    x2(g)=x;
                    xr2(g,:,:)=xr;
                    transdata2(g)=transdata;%Mbits
                    starttime=endtime;
                    qoe2=qoe+qoe2;
                    pausetime2=pausetime+pausetime2;
                    pausecount2=pausecount+pausecount2;
                end
                 
                decodetime2=0;
                for g=1:G
                    for k=1:GOF_K(g)
                        for r=1:R
                            decodetime2=decodetimeTile(g,k,r)*xr2(g,k,r)*cv(g,k)+decodetime2;
                        end
                    end
                end
                xv2=sum(x2)/G;
                decodetime2=decodetime2/NC(nc);
                %qoe2=A*xv2-B*pausetime2+D;
                %--------------------------------------------------------------------------------------------------------------------------------------------------------------

                %-----------------------------------------------------------------------------------------联合资源传输方案--------------------------------------------------------
                x3=zeros(1,G);
                xr3=zeros(G,K,R);
                pausetime3=0;
                pausecount3=0;
                b=b0*1/3;
                flag=1;%播放or缓冲
                qoe3=0;
                starttime=1;
                transdata3=zeros(1,G);
                for g=1:G
                    %downloadtime3(g)=starttime;
                    
                    [x,e,pausetime,pausecount,b,xr,flag,qoe,endtime,transdata]=TileSelect(cv,plysize,decodetimeTile,BW(bw,:),3,b0,NC(nc),b,g,binsize,GOF_K,flag,starttime);
                    starttime=endtime;
                    qoe3=qoe+qoe3;
                    x3(g)=x;
                    transdata3(g)=transdata;%Mbits
                    xr3(g,:,:)=xr;
                    pausetime3=pausetime+pausetime3;
                    pausecount3=pausecount+pausecount3;
                    e3(g,:)=e;
                end
                xv3=sum(x3)/G;
                
                decodetime3=0;
                for g=1:G
                    for k=1:GOF_K(g)
                        for r=1:R
                            decodetime3=decodetimeTile(g,k,r)*e3(g,k)*xr3(g,k,r)*cv(g,k)+decodetime3;%Mbits
                        end
                    end
                end
                decodetime3=decodetime3/NC(nc);
                %qoe3=A*xv3-B*pausetime3+D;
                %--------------------------------------------------------------------------------------------------------------------------------------------------------------
                filename=['Ver' num2str(Version) 'coder' num2str(coder) 'Se' num2str(s) 'TileScheme' num2str(TileScheme) 'BW' num2str(bw) 'NC' num2str(nc) '.mat'];
                %save(filename,'plysize','binsize','decodetimeTile','cv','delaytime3','pausetime3','pausecount3','x3','xv3','xr3','e3');
                save(filename,'pausetime1','pausetime2','pausetime3','pausetime5','x2','xv2','xr2','x3','xv3','xr3','e3','transdata1','transdata2','transdata3','transdata5','decodetime1','decodetime2','decodetime3','decodetime5','qoe1','qoe2','qoe3','qoe5');
           end
        end
    end
end
end
