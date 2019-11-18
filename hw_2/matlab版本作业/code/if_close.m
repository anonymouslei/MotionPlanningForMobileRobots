function flag = if_close(CLOSED,CLOSEDCOUNT,xval,yval)
    %This function returns whethe the node in the OPEN
    %
    i=1;
    while(i <= CLOSEDCOUNT)
        if (CLOSED(i,1) == xval & CLOSED(i,2) == yval)
            break;
        end
        i = i + 1;
    end
    if (i <= CLOSEDCOUNT)
        flag = 1;
    else
        flag = 0;
end