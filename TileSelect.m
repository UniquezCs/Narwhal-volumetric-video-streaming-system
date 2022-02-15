function [x,e,pausetime,pausecount,b,xr,flag,qoe,endtime,transdata] = TileSelect(cv,plysize,decodetimeTile,Bw,scheme,b0,NC,b,g,binsize,GOF_K,flag,starttime)
%TILESELECT 此处显示有关此函数的摘要

G=size(decodetimeTile,1);
K=size(decodetimeTile,2);%
R=size(decodetimeTile,3);%
tilev=sum(cv(g,:));
e=zeros(1,K);
x=zeros(K,R);
xr=0;
b0=b0*1/3;
pausecount=0;
A=1;B=18;D=10;
endtime=1;
transdata=0;
%--------------------------------------------------非自适应--------------------------------------------------------
if scheme ==1
    x = zeros(K,R);
    x(:,1)=1;
    x(find(cv(g,:)==0),:)=0;
    %传输时间
    ts=0;
    for k=1:GOF_K(g)
        for r=1:R
          ts=binsize(g,k,r)*x(k,r)*cv(g,k)/Bw(g)+ts;
        end
    end   
%     endtime=DownloadTime(starttime,Bw,ts_ply);
%     ts=(endtime-starttime)/3;%实际时间
    %解码时间
    td=0;
    for k=1:GOF_K(g)
        for r=1:R
            td=decodetimeTile(g,k,r)*x(k,r)*cv(g,k)/NC+td;
        end
    end      
    t=td+ts;
    %质量
    q=0;
    for k=1:GOF_K(g)
        for r=1:R
          q=x(k,r)*r+q;
        end
    end
    q=q/tilev;
    x=1;
    pausetime=max(t-b-1/3,0);
    b=max(b-t+1/3,0);
    
%     pausetime=0;
%     if b>b0*1/5 &&flag==1
%         b=max(b-t+1/3,0);
%         flag=1;
%     elseif b<b0*1/10 &&flag==1 %第一次进缓冲状态
%         pausetime=t;
%         b=b+1/3;
%         flag=0;
%     elseif (b<b0*1/2) &&flag==0
%         pausetime=t;
%         b=b+1/3;
%         flag=0;
%     elseif b>b0*1/4 &&flag==0
%         pausetime=t;
%         b=b+1/3;
%         flag=1;
%     end
    
    qoe=A*q-B*pausetime+D; 
%---------------------------------------------------------------------------------------
elseif scheme ==5
    x = zeros(K,R);
    x(:,5)=1;
    x(find(cv(g,:)==0),:)=0;
    %传输时间
    ts=0;
    for k=1:GOF_K(g)
        for r=1:R
          ts=binsize(g,k,r)*x(k,r)*cv(g,k)/Bw(g)+ts;
        end
    end
    
%     endtime=DownloadTime(starttime,Bw,ts_ply);
%     ts=(endtime-starttime)/3;%实际时间
       
    %解码时间
    td=0;
    for k=1:GOF_K(g)
        for r=1:R
            td=decodetimeTile(g,k,r)*x(k,r)*cv(g,k)/NC+td;
        end
    end        
    t=td+ts;
    
    %质量
    q=0;
    for k=1:GOF_K(g)
        for r=1:R
          q=x(k,r)*r+q;
        end
    end
    q=q/tilev;
    x=5;
    %pausetime=max(t-b-1/3,0);
    b=max(b-t+1/3,0);
    pausetime=max(t-b-1/3,0);
   
%     if b>b0*1/5 &&flag==1
%         b=max(b-t+1/3,0);
%         flag=1;
%     elseif b<b0*1/10 &&flag==1 %第一次进缓冲状态
%         pausetime=t;
%         b=b+1/3;
%         flag=0;
%     elseif (b<b0*1/2) &&flag==0
%         pausetime=t;
%         b=b+1/3;
%         flag=0;
%     elseif b>b0*1/4 &&flag==0
%         pausetime=t;
%         b=b+1/3;
%         flag=1;
%     end
    qoe=A*q-B*pausetime+D;

%------------------------------------------------------传统自适应--------------------------------------------------------------------------
elseif scheme==2
    x = binvar(K,R);
%     x=zeros(K,R);
%     x(:,1)=1;
%     x(find(cv(g,:)==0),:)=0;
    C = [];
    e=ones(1,K);
    for k=find(cv(g,:)~=0)
    s = sum(x(k,:));
    C = [C;s==1];
    end
    for k=find(cv(g,:)==0)
    s = sum(x(k,:));
    C = [C;s==0];
    end
    
    %传输时间  
    
    ts_ply=0;
    for k=1:GOF_K(g)
        for r=1:R
          ts_ply=binsize(g,k,r)*x(k,r)*cv(g,k)/Bw(g)+ts_ply;
        end
    end
    
    %ts_ply=sum(sum(binsize(g,:,:).*x(:,:).*cv(:,:)))/Bw(g);  
    %解码时间
    td=0;
    for k=1:GOF_K(g)
        for r=1:R
            td=decodetimeTile(g,k,r)*x(k,r)*cv(g,k)/NC+td;
        end
    end       
    t=td+ts_ply;
    %质量
    q=0;
    for k=1:GOF_K(g)
        for r=1:R
          q=x(k,r)*r+q;
        end
    end   
    q=q/tilev;
    z=A*q-B*max(t-b-1/3,0)+D;
    z=-z;
    % 参数设置
    options = sdpsettings('verbose',2,'solver','GUROBI');
    %options.mosek.MSK_IPAR_INTPNT_MAX_ITERATIONS=10;
    options.gurobi.MIPGapAbs=0.05;
    % 求解
    result  = optimize(C,z,options);
    xr=value(x);
    x=value(x);
    qoe=value(-z);
    xx=0;
    t=value(t);
    td=value(td);
    ts_ply=value(ts_ply);
    transdata=0;
    for k=1:GOF_K(g)
        for r=1:R
            transdata=binsize(g,k,r)*x(k,r)*cv(g,k)+transdata;%Mbits
        end
    end  
%     endtime=DownloadTime(starttime,Bw,transdata);
%     ts=(endtime-starttime)/3;%实际时间
%     t=value(td)+ts;
    for k=1:GOF_K(g)
        for r=1:R
          xx=x(k,r)*r+xx;
        end
    end
    xx=xx/tilev;
    x=xx;
    %pausetime=max(t-b-1/3,0); 
    b=max(b-t+1/3,0);
    pausetime=max(t-b-1/3,0);
%     if b>b0*1/5 &&flag==1
%         b=max(b-t+1/3,0);
%         flag=1;
%     elseif b<b0*1/10 &&flag==1 %第一次进缓冲状态
%         pausetime=t;
%         b=b+1/3;
%         flag=0;
%     elseif (b<b0*1/2) &&flag==0
%         pausetime=t;
%         b=b+1/3;
%         flag=0;
%     elseif b>b0*1/4 &&flag==0
%         pausetime=t;
%         b=b+1/3;
%         flag=1;
%     end
    %qoe=A*value(q)-B*pausetime+D;
% %********************************************************************************************************************************************************   

%******************************联合资源下的自适应************************************************************************************************************    
elseif scheme==3
    %e = binvar(1,K);
    %e=ones(1,K);
    x=binvar(K,R,2);
    C = [];
    for k=find(cv(g,:)~=0)
    s = sum(sum(x(k,:,:)));
    C = [C;s==1];
    end
    for k=find(cv(g,:)==0)
    s = sum(sum(x(k,:,:)));
    C = [C;s==0];
    end
    
%     for k=find(cv(g,:)==0)
%     s = e(k);
%     C = [C;s==0];
%     end

    ts_ply=0;
    %非压缩形式的时间
    for k=1:GOF_K(g)
        for r=1:R
            ts_ply=plysize(g,k,r)*x(k,r,1)*cv(g,k)/Bw(g)+ts_ply;
        end
    end

     %压缩形式的传输时间
    ts_bin=0;
    for k=1:GOF_K(g)
        for r=1:R
            ts_bin=binsize(g,k,r)*x(k,r,2)*cv(g,k)/Bw(g)+ts_bin;
        end
    end
    
    td=0;
     
    %压缩形式的解码时间
    for k=1:GOF_K(g)
        for r=1:R
            td=decodetimeTile(g,k,r)*x(k,r,2)*cv(g,k)/NC*cv(g,k)+td;
        end
    end
    
    t=max(ts_ply+ts_bin,ts_bin+td);
    %t=ts_ply+ts_bin+td;
    q=0;
    for k=1:GOF_K(g)
        for r=1:R
            for e=1:2
                q=x(k,r,e)*r+q;
            end
        end
    end
    
    q=q/tilev;
    z=A*q-B*max(t-b-1/3,0)+D;

    z=-z;
    % 参数设置
    options = sdpsettings('verbose',2,'solver','GUROBI');
    %options.mosek.MSK_IPAR_INTPNT_MAX_ITERATIONS=10;
    options.gurobi.MIPGapAbs=0.5;
    %求解
    result  = optimize(C,z,options);
    x=value(x);
    xr=value(x);
    e=reshape(sum(x(:,:,:),2),K,2)';
    e=e(1,:);
    xr=sum(xr(:,:,:),3);
    
    ts_ply=value(ts_ply);
    ts_bin=value(ts_bin);
    td=value(td);
    t=value(t);
     qoe=value(-z);
    %实际传输时间
    transdata=0;   
    for k=1:GOF_K(g)
        for r=1:R
            transdata=binsize(g,k,r)*e(k)*x(k,r)*cv(g,k)+transdata;%Mbits
        end
    end
%     endtime=DownloadTime(starttime,Bw,transdata);
%     ts_bin=(endtime-starttime)/3;
%     starttime=endtime;
    transdata=0; 
    for k=1:GOF_K(g)
        for r=1:R
            transdata=binsize(g,k,r)*e(k)*x(k,r)*cv(g,k)+plysize(g,k,r)*(1-e(k))*x(k,r)*cv(g,k)+transdata;%Mbits
        end
    end
%     endtime=DownloadTime(starttime,Bw,transdata);
%     ts_ply=(endtime-starttime)/3;
    
    %实际总时间
   % t=value(td)+ts_bin+ts_ply;   
 
    x=value(q);
    for k=1:K
    %e(k)=find(xr(k,:,:);
    end
    b=max(b-t+1/3,0); 
    pausetime=max(t-b-1/3,0);
%     if b>b0*1/5 &&flag==1
%         b=max(b-t+1/3,0);
%         flag=1;
%     elseif b<b0*1/10 &&flag==1 %第一次进缓冲状态
%         pausetime=t;
%         b=b+1/3;
%         flag=0;
%     elseif (b<b0*1/2) &&flag==0
%         pausetime=t;
%         b=b+1/3;
%         flag=0;
%     elseif b>b0*1/4 &&flag==0
%         pausetime=t;
%         b=b+1/3;
%         flag=1;
%     end
%     qoe=A*value(q)-B*pausetime+D;
end