function show_matches(image1, image2, f1, f2, matches, inliers)
    % matches - all SIFT mathces 
    % inliers - array of indicies of inliers
    numMatches = size(matches,2);
    
    figure(2) ; clf ;
    subplot(2,1,1) ;
    imagesc([image1 image2]) ;

    offset = size(image1,2) ;
    line([f1(1,matches(1,:));f2(1,matches(2,:))+offset], ...
         [f1(2,matches(1,:));f2(2,matches(2,:))]) ;
    title(sprintf('%d tentative matches', numMatches)) ;
    axis image off ;

    subplot(2,1,2) ;
    imagesc([image1, image2]) ;
    offset = size(image1,2) ;
    line([f1(1,matches(1,inliers));f2(1,matches(2,inliers))+offset], ...
         [f1(2,matches(1,inliers));f2(2,matches(2,inliers))]) ;
    title(sprintf('%d (%.2f%%) inliner matches out of %d', ...
                  length(inliers), ...
                  100*length(inliers)/numMatches, ...
                  numMatches)) ;
    axis image off ;

    drawnow ;
end