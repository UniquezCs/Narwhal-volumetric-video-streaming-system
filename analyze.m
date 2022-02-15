%
clear;
load BW.mat
load view.mat
load points.mat

G=150;
decodetime=zeros(4,4,2,2,4);

e1=zeros(4,2,2,4,G,1);

est=zeros(4,2,2,4,G,12);

pausetime=zeros(4,4,2,2,4);
quality=zeros(4,4,2,2,4);
qoe=zeros(4,4,2,2,4);
transdata=zeros(4,4,2,2,4,G);
XR4=zeros(4,2,2,4,G,12,5); %语义切分
ratio=[7,400];
decode_ratio=[1,10];
for coder=1:2
for s=1:2
    for Ti=[1,4] %切块方案:1,12,36,语义切分
        for nc=1
            for bw=1:2
                filename=['Ver7' 'coder' num2str(coder) 'Se'  num2str(s) 'TileScheme' num2str(Ti) 'BW' num2str(bw) 'NC' num2str(nc) '.mat'];
                load(filename);
                
                decodetime(Ti,1,coder,bw,s)=decodetime1;
                decodetime(Ti,2,coder,bw,s)=decodetime2;
                decodetime(Ti,3,coder,bw,s)=decodetime3;
                decodetime(Ti,4,coder,bw,s)=decodetime5;
                if Ti==1
                e1(Ti,coder,bw,s,:,:)=e3;
                elseif Ti==2
                    e12(Ti,coder,bw,s,:,:)=e3;
                elseif Ti==3
                    e36(Ti,coder,bw,s,:,:)=e3;
                else
                    est(Ti,coder,bw,s,:,:)=e3;
                end
                pausetime(Ti,1,coder,bw,s)=pausetime1;
                pausetime(Ti,2,coder,bw,s)=pausetime2;
                pausetime(Ti,3,coder,bw,s)=pausetime3;
                pausetime(Ti,4,coder,bw,s)=pausetime5;
                           
                qoe(Ti,1,coder,bw,s)=qoe1;
                qoe(Ti,2,coder,bw,s)=qoe2;
                qoe(Ti,3,coder,bw,s)=qoe3;
                qoe(Ti,4,coder,bw,s)=qoe5;
                
                quality(Ti,1,coder,bw,s)=1;
                quality(Ti,2,coder,bw,s)=xv2;
                quality(Ti,3,coder,bw,s)=xv3;
                quality(Ti,4,coder,bw,s)=5;
                               
                if Ti==1
                    cv=cv1;
                    pointN=point_1_1;
                    K=1;
                elseif Ti==2
                    cv=cv2;
                    pointN=point_2_3;
                    K=12;
                elseif Ti==3
                    cv=cv3;
                    pointN=point_3_4;
                    K=36;
                elseif Ti==4
                    cv=cv4;
                    pointN=point_smt;
                    K=12;
                end
                
                
                plysize=zeros(G,K,5);%每个切块序列未压缩文件大小
                
                %ply文件大小              
                plysize(:,:,:)=pointN(s,:,:,:)*17/1024/1024*8;
                %压缩文件大小
                binsize=plysize/ratio(coder);
               
                transdata3=zeros(1,G);
                
                for g=1:G
                    tmp=0;
                for k=1:K
                    for r=1:5
                        tmp=tmp+binsize(g,k,r)*(1-e3(g,k))*xr3(g,k,r)*cv(g,k)+plysize(g,k,r)*e3(g,k)*xr3(g,k,r)*cv(g,k);                     
                    end
                end
                transdata3(g)=tmp;
                end
                
                                            
                transdata(Ti,1,coder,bw,s,:)=transdata1;
                transdata(Ti,2,coder,bw,s,:)=transdata2;
                transdata(Ti,3,coder,bw,s,:)=transdata3;
                transdata(Ti,4,coder,bw,s,:)=transdata5;
                           %XR4=zeros(4,2,2,4,G,12,5); %语义切分
                           if Ti==4
                XR4(1,coder,bw,s,:,:,:)=zeros(G,12,5);
                XR4(1,coder,bw,s,:,:,1)=1;
                XR4(2,coder,bw,s,:,:,:)=xr2;
                XR4(3,coder,bw,s,:,:,:)=xr3;
                XR4(4,coder,bw,s,:,:,:)=zeros(G,12,5);
                XR4(4,coder,bw,s,:,:,5)=1;
                           end                         
            end
        end
    end
end
end
%平均传输量随带宽变化:data_bw 4x5x150 
%4
%5:4(2)种方案和带宽
%150:GoF时间
data_bw=zeros(4,5,150);

%QoE比较      QoE_comp:2x4x4x4
QoE_comp=zeros(2,4,4,4);
% e1=zeros(4,2,2,4,G,1);
% e12=zeros(4,2,2,4,G,12);
% e36=zeros(4,2,2,4,G,36);
% est=zeros(4,2,2,4,G,12);
% pausetime=zeros(4,4,2,2,4);
% qoe=zeros(4,4,2,2,4);
% transdata=zeros(4,4,2,2,4,G); 切块(1,12,36,st) 传输(1,AT,JT,5) nc bw 序列

%平均传输量随带宽变化:data_bw 4x5x150
%4:Co1BW1传输,Co1BW1切块,Co2BW1传输,Co2BW1切块
%5:4种方案和带宽
%150:GoF时间
data_bw(1,1:4,:)=sum(transdata(4,:,1,1,:,:),5)/2;
data_bw(1,5,:)=BW(1,:);

data_bw(2,1:4,:)=sum(transdata(:,2,1,1,:,:),5)/2;
data_bw(2,3,:)=BW(1,:);

data_bw(3,1:4,:)=sum(transdata(4,:,2,2,:,:),5)/2;
data_bw(3,5,:)=BW(1,:);

data_bw(4,1:4,:)=sum(transdata(:,2,2,2,:,:),5)/2;
data_bw(4,3,:)=BW(1,:);

%QoE比较      QoE_comp:2x4x4x4

QoE_comp(1,:,1,1)=qoe(4,:,1,1,1);
QoE_comp(1,:,2,1)=qoe(4,:,1,2,1);
QoE_comp(1,:,3,1)=qoe(4,:,2,1,1);
QoE_comp(1,:,4,1)=qoe(4,:,2,2,1);

QoE_comp(2,:,1,1)=qoe(:,3,1,1,1);
QoE_comp(2,:,2,1)=qoe(:,3,1,2,1);
QoE_comp(2,:,3,1)=qoe(:,3,2,1,1);
QoE_comp(2,:,4,1)=qoe(:,3,2,2,1);

QoE_comp=(QoE_comp+3000)/1000;


decodetime_comp(1,:,1,1)=decodetime(4,:,1,1,1);
decodetime_comp(1,:,2,1)=decodetime(4,:,1,2,1);
decodetime_comp(1,:,3,1)=decodetime(4,:,2,1,1);
decodetime_comp(1,:,4,1)=decodetime(4,:,2,2,1);

decodetime_comp(2,:,1,1)=decodetime(:,3,1,1,1);
decodetime_comp(2,:,2,1)=decodetime(:,3,1,2,1);
decodetime_comp(2,:,3,1)=decodetime(:,3,2,1,1);
decodetime_comp(2,:,4,1)=decodetime(:,3,2,2,1);


