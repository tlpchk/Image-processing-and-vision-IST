
function [R1_to_2, T1_to_2] = rigid_transform(image1, image2, xyz1, xyz2)
%transformation - Funcao que, ao receber o par de imagens, produz a matriz
% R e o vector T, que representam a transformacao dos pontos 3D da camara
% de profundidade da primeira imagem para o referencial da segunda.
% Recorre-se a biblioteca vl_setup para a aplicacao do algoritmo SIFT.
%
%  Argumentos: 
%   - image 1: primeira imagem do par da camara rgb;
%   - image 2: segunda imagem do par da camara rgb;
%   - xyz1: pontos 3D obtidos pela camara de profundiade da image1
%   - xyz2: pontos 3D obtidos pela camara de profundiade da image2
%
%  Saida:
%   - A matriz R e o vector T descrevem a transformacao dos pontos 3D da  
% camara de profundidade da imagem1 no sistema de coordenadas da imagem2.

%% Inicializacao
% Numero de pontos a realizar a correspondencia entre as 2 imagens
nr_pontos = 4;

% Valor do treshold para calculo do erro
tresh = 0.4;

% Numero de repeticoes para os conjuntos de 4 pontos
nr_repeticoes = 1000;

% Inicializacao de inliers e do numero de inliers
best_inliers = 0;
best_nr_inliers = 0;

%% SIFT
[f1,d1] = vl_sift(single(rgb2gray(image1)));
[f2,d2] = vl_sift(single(rgb2gray(image2)));
[matches, scores] = vl_ubcmatch(d1, d2);

% (Filtragem) Retira valores com um score acima de 20.000 - considerados muito maus
index=find(scores<20000);
matches=matches(:,index');

%% RANSAC
% Conjuntos de 4 pontos escolhidos aleatoriamente dos pontos da matriz 
% matches, ou seja, de 4 features comuns as duas imagens, para computar a 
% matriz de rotacao e o vector translacao

for i=1:nr_repeticoes
    %% Escolha aleatoria de 4 pontos da matriz matches
    dots = randperm(length(matches),nr_pontos); % Para nao se repetirem a sequencia dos pontos escolhidos aleatoriamente
    u1 = f1(1,matches(1,dots));
    v1 = f1(2,matches(1,dots));
    u2 = f2(1,matches(2,dots));
    v2 = f2(2,matches(2,dots));

    %% Calculo dos centroides com base nesses 4 pontos
    ind_dots1 = sub2ind([480 640],uint64(v1),uint64(u1));
    ind_dots2 = sub2ind([480 640],uint64(v2),uint64(u2));
    cent1 = mean(xyz1(ind_dots1,:))';
    cent2 = mean(xyz2(ind_dots2,:))';
    pc1 = xyz1(ind_dots1,:)'-repmat(cent1,1,length(u1));
    pc2 = xyz2(ind_dots2,:)'-repmat(cent2,1,length(u2));

    %% Calculo da transformacao (R+T), referente aos 4 pontos
    [a b c] = svd(pc2*pc1');
    R12 = a*c';
    T12 = cent2-R12*cent1;

    %% Obtencao dos pares (x,y) de todos os pontos da matriz matches, de ambas as imagens
    x1 = f1(1,matches(1,:));
    y1 = f1(2,matches(1,:));
    x2 = f2(1,matches(2,:));
    y2 = f2(2,matches(2,:));

    ind1 = sub2ind([480 640],uint64(y1),uint64(x1));
    ind2 = sub2ind([480 640],uint64(y2),uint64(x2));

    %% Calculo do erro entre os pontos da imagem 1 e os pontos da imagem 2, tendo em conta a aplicacao da transformacao aos pontos da imagem 1
    error_eq = xyz2(ind2,:)-((xyz1(ind1,:))*R12+repmat(T12',length(matches),1));
    error = sqrt(sum(error_eq.^2,2));

    %% Identificacao dos inliers, atraves do cumprimento do treshold
    inliers = find(error<tresh);
    if isempty(inliers)
        inliers = [];
    end
    
    % Para que sejam bons inliers, tem que ser pelo menos metade dos pontos
    % correspondentes entre as duas imagens
    if ((length(inliers) >= round(0.5*length(matches))) && (length(inliers) >= best_nr_inliers))
        best_inliers = inliers; % Indice dos melhores inliers encontrados
        best_nr_inliers = length(inliers); % Maior numero de inliers encontrados
    end
end
%% Escolha de todos os pontos inliers da matriz matches de forma a obter a 
% matriz de Rotacao e o vector de Translacao finais                                                
u1 = f1(1,matches(1,best_inliers));
v1 = f1(2,matches(1,best_inliers));
u2 = f2(1,matches(2,best_inliers));
v2 = f2(2,matches(2,best_inliers));

%% Calculo dos centroides com base em todos os inliers
ind1 = sub2ind([480 640],uint64(v1),uint64(u1));
ind2 = sub2ind([480 640],uint64(v2),uint64(u2));
cent1 = mean(xyz1(ind1,:))';
cent2 = mean(xyz2(ind2,:))';
pc1 = xyz1(ind1,:)'-repmat(cent1,1,length(u1));
pc2 = xyz2(ind2,:)'-repmat(cent2,1,length(u2));

%% Calculo da transformacao final, tendo em conta os melhores inliers
[a b c] = svd(pc2*pc1');
R1_to_2 = a*c';
T1_to_2 = cent2-R1_to_2*cent1;
end