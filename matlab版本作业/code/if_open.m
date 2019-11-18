function flag = if_open(OPEN,xval,yval)
    %This function returns whether the node in the OPEN
    %
    i=1;
    while(i <= size(OPEN,1))
        if (OPEN(i,2) == xval && OPEN(i,3) == yval && OPEN(i,1) == 1)
            break;
        end
        i = i + 1;
    end
    if (i <= size(OPEN,1))
        flag = 1;
    else
        flag = 0;
end