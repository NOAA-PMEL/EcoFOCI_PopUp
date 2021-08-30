for(numFiles=1:1:128)
    Order = zeros(numFiles,3);
    OrderUsed(1:numFiles,1)=false;
    Order(1,1)=1;
    Order(1,2)=0;
    Order(2,1)=numFiles;
    Order(2,2)=0;
    OrderUsed(1)=true;
    OrderUsed(numFiles,1)=true;
    divisor=2;
    OrderMark=3;
    pass=-1;
    while(numFiles/divisor>1)
        for(step=2:2:divisor)
            selectImage=floor(numFiles/divisor)+ceil((step-2)*(numFiles/divisor));
            if(OrderUsed(selectImage)==false)
                Order(OrderMark,1)=selectImage;
                Order(OrderMark,2)=pass;
                OrderUsed(selectImage,1)=true;
                OrderMark=OrderMark+1; 
            end
        end
        pass=pass-1;
        divisor=divisor*2;
    end

    for(fillHoles=1:1:numFiles)
        if(OrderUsed(fillHoles)==false)
            Order(OrderMark,1)=fillHoles;
            Order(OrderMark,2)=pass;
            OrderMark=OrderMark+1;   
        end
        Order(fillHoles,3)=fillHoles;
    end

    scatter(Order(:,1),Order(:,2))
    xlabel('Image Number');
    ylabel('Algorithm Pass');
    b = num2str(Order(:,1)); 
    c = cellstr(b);
    dx = 0.1; dy = -0.1;
    text(Order(:,1)+dx,Order(:,2)+dy,c)
    b = num2str(Order(:,3)); 
    c = cellstr(b);
    dx = 0.1; dy = 0.1;
    text(Order(:,1)+dx,Order(:,2)+dy,c)
    ylim([pass-1 1]);
    xlim([0 numFiles+1 ]);
    pause(.1);
end