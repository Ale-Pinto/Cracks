%% Lettura immagini

crepa = imread("DJI_0832.tif");  % DJI_0828 e 0829

load("target_mano_cal.mat"); % lettura parametri della camera

crepau = undistortImage(crepa, cameraParams); % rimozioni distorsione camera

crepag = rgb2gray(crepau); % passaggio ad immagini scala di grigi
%% Miglioramento del contrasto delle immagini

% Passaggio utile per migliorare la procedura di binarizzazione,
% rendendo la fessura più evidente rispetto allo sfondo. Testati diversi metodi per
% migliorare il contrasto.

crepa_adj = imadjust(crepag); % equalizzazione istogrammi semplice satura l'1% inferiore e l'1% superiore di tutti i valori dei pixel

crepa_hist = adapthisteq(crepag); % equalizzazione istogrammi adattiva usando contrast-limited adaptive histogram equalization (CLAHE)
 
imshowpair(crepa_adj, crepa_hist, 'montage')
%% Filtraggio del rumore di fondo

% Testati diversi filtri per eliminare parte del rumore di fondo presente nelle
% immmagini.

crepa_wiener = wiener2(crepa_hist, [3 3]); % Filtro di Wiener, possibilità di cambiare la finestra con la quale filtrare. Preserva meglio i bordi
crepa_wiener_ad = imadjust(crepa_wiener);
imshow(crepa_wiener_ad)

% crepa_med = medfilt2(crepa_adj,[5 5]); % Filtro mediano, possibilità di cambiare finestra filtraggio
% crepa_med_ad = imadjust(crepa_med);
% % imshow(crepa_med_ad)
% 
% imshowpair(crepa_med_ad, crepa_wiener_ad, 'montage')

% Possibilità di usare altri filtri, come la media, andando a crearli ad hoc
% avg = fspecial("average",[5 5]);
% crepa_avg = imfilter(crepa_hist,avg);
% montage({crepa_wiener, crepa_avg, crepa_med})

%% Binarizzazione immagine e operatori morfologici

% Possibilità di cambiare algoritmo di binarizzazione (Sauvola, Niblack). Attualmente usa
% metodo adattivo di Bradley.

bw= imbinarize(crepa_wiener_ad, 'adaptive', 'Sensitivity', 0.01 ,'ForegroundPolarity', 'dark'); % "Sensitivity" più valore alto più rumore di fondo viene binarizzato male. 
bw_flip = ~bw; % Se si usa "global" al posto di adaptive usa il metodo di Otsu usando una sola threshold per tutta l'immagine.
imshow(bw_flip)

% Creazione di una maschera manuale per limitare la zona della fessura e
% velocizzare il riconoscimento rispetto al rumore di fondo.
imshow(bw_flip)
contorni = drawfreehand;
maschera = createMask(contorni, bw_flip);
imshow(maschera)
BW = bw_flip == 1 & maschera ==1; % cambiare i valori in base al tipo di maschera.
imshow(BW)
contorni = drawfreehand;
maschera = createMask(contorni, BW);
imshow(maschera)
BW2 = BW == 1 & maschera == 0; % cambiare i valori in base al tipo di maschera.
imshow(BW2)

% Utilizzo di filtri morfologici per eliminare il rumore. Diversi approcci,
% in genere area e extent i più selettivi.

bw_filt = bwpropfilt(BW,'Area',[50 1000000000]);
figure;
imshow(bw_filt)

bw_filt2 = bwpropfilt(bw_filt, 'extent', [0 0.4]);
figure;
imshow(bw_filt2)

bw_filt3 = bwpropfilt(bw_filt, 'perimeter', [400 10000000]);
imshow(bw_filt3)

bw_filt4 = bwpropfilt(bw_filt3, 'area', [1000 50000]);

% Possibile ulteriore maschera per rifinire i confini della fessura.
imshow(bw_filt2)
contorni = drawfreehand;
maschera = createMask(contorni, bw_filt);
imshow(maschera)
BW = bw_filt == 1 & maschera ==1; % cambiare i valori in base al tipo di maschera.
imshow(BW)
contorni = drawfreehand;
maschera = createMask(contorni, BW_2)
BW_2 = BW_2 == 1 & maschera ==0; % cambiare i valori in base al tipo di maschera.
imshow(BW_2)
%% Ricerca scheletro

% Utilizzo algoritmo di scheletonizzazione per estrarre lo scheletro della
% fessura. Prima vengono usati altri operatori morfologi per rimuovere artifici
% visivi e buchi all'interno fessura binarizzata

bw_spur = bwmorph(bw_filt,'spur'); % Elimina pixel spuri, pixel singoli che sporgono rispetto al bordo continuo.

bw_fill = imfill(bw_spur, 'holes'); % Riempie i buchi nella fessura binarizzata.

bw_spur2 = bwmorph(bw_fill,'spur');

shape = strel('disk', 5); % Chiusura dell'immagine, è possibile modificare l'elemento "shape" con altre forme e dimensioni.
bw_close = imclose(bw_spur, shape);
bw_fill2 = imfill(bw_close, 'holes');
imshow(bw_close)

skel = bwskel(bw_close, 'MinBranchLength' , 20); % MinBranchLength elimina i rami troppo corti che si ottengono in zone particoalrmente aperte della fessura.
figure;
imshow(skel)
figure;
imshow(bw_fill2)
%% Ricerca bordi

% I bordi possono essere estratti usando altre maschere come "Sobel" o
% "Prewitt", cambia molto poco il risultato per via che l'immgaine di partenza è binarizzata. 

[bw_edge, thresh] = edge(bw_close, "canny"); % Ricerca con "Canny"
imshow(bw_edge)
%% Associazione scheletro-bordi

% Associazione per ciascun pixel dello scheletro dei due pixel dei bordi
% vicini, per valutare l'apertura. I bordi sono cercati nella direzione
% perpendicolare allo scheletro. Si effettua la ricerca ruotando lo
% scheletro e i bordi. I diversi If dipendono dal trend locale dello
% scheletro, in quanto in alcuni casi la rotazione diventa non necessaria.
% Cambia il procedimento in base al tipo di rotazione da applicare.

q = 20; % Dimensione finestra di ricerca in immagine bordi.

p = 1; % Dimensione finestra di ricerca in immagine sceheletro.

gauss_std = 2; % Valore per fitting gaussiano nella ricerca dei pixel dei bordi

s = struct(); % Struttura per salvare coordinate immagine X e Y di scheletro, bordo destro e bordo sinistro
s.X_skel = 0;
s.Y_skel= 0;
s.X_edge_s = 0;
s.Y_edge_s= 0;
s.X_edge_d = 0;
s.Y_edge_d= 0;


k=1; % indice per proseguire il ciclo

% maschere per isolare e ricercare i bordi
a = zeros(2*q+1,q);
b = ones(2*q+1,q+1);
z = zeros(q,2*q+1);
d = ones(q+1,2*q+1);
v = ones([2*q+1 1]);
h = ones([1 2*q+1]);
sins = [b,a]; % Isola il bordo a sinistra del pixel centrale della finestra.
des = [a,b]; % Isola il bordo a destra del pixel centrale della finestra.
su = [d;z]; % Isola il bordo sopra il pixel centrale della finestra.
giu = [z;d]; % Isola il bordo sotto il pixel centrale della finestra.
mask_v = zeros([2*q+1 2*q]);
mask_h = zeros([2*q 2*q+1]);
mask_v = [mask_v(:,1:q),v,mask_v(:,q+1:end)]; % Maschera di ricerca verticale.
mask_h = [mask_h(1:q,:);h;mask_h(q+1:end, :)]; % Maschera di ricerca orizzontale.
uscita = zeros(100000, 2);

% Aggiunta bordi neri alle immagini
bord = zeros(size (skel)+(2*q), 'like', skel);
bord(q+1:size(skel,1)+q,q+1:size(skel,2)+q)=skel;
bord_edg = zeros(size (bw_edge)+(2*q), 'like', bw_edge);
bord_edg(q+1:size(bw_edge,1)+q,q+1:size(bw_edge,2)+q)=bw_edge;

for i = q+1:size(bord,1)-q
   for j = q+1:size(bord,2)-q
       if bord(i,j) == 1 % condizione per considerare solo i pixel dello scheletro
%           i = 3975;
%           j = 1774;
%           lastwarn('')
          IA = uint8(bord(i-p:i+p,j-p:j+p));% apertura finestra in immagine scheletro centrata su pixel scheletro
          [y,x] = find(IA);
          c = polyfit(x,y,1); % ricerca andamento lineare locale scheletro
          angle = atand(c(1));
%           msg = lastwarn;
%           if ~isempty(msg)
%               uscita = [i j];
%                continue
%           end
          if angle >= -10 && angle <= 10 % Scheletro orizzontale, ricerca verticale no rotazione
             SA = double(bord_edg(i-q:i+q,j-q:j+q)); % apertura finestra in immagine bordi centrata su pixel scheletro
             SA_s = su.*SA; % ricerca pixel bordo superiore
             temp_s= SA_s + mask_v;
             temp_s = temp_s == 2;
             ind_s = find(temp_s,1);
             if length(ind_s) < 1 % Salta se non trova bordi
                continue
             end
             [y_s,x_s] = ind2sub(size(SA_s),ind_s);
             u_s = [x_s-(q+1) y_s-(q+1)]; % vettore bordo sinistro
             SA_g = giu.*SA; % ricerca pixel bordo inferiore
             temp_g = SA_g + mask_v;
             temp_g = temp_g == 2;
             ind_g = find(temp_g,1);
             if length(ind_g) < 1 % Salta se non trova bordi
                continue
             end
             [y_d,x_d] = ind2sub(size(SA_g),ind_g);
             u_d = [x_d-(q+1) y_d-(q+1)]; % vettore bordo destro
             s(k).Y_skel = i-q;
             s(k).X_skel = j-q;
             s(k).X_edge_s = j - q; % Riposiziona le coordinate nel sistema immagine corretto
             s(k).Y_edge_s= i - q - norm(u_s);
             s(k).X_edge_d =j - q;
             s(k).Y_edge_d= i - q + norm(u_d);
             k = k+1;
          elseif angle <= -80 || angle >= 80 % Scheletro verticale, ricerca orizzontale no rotazione
             SA = double(bord_edg(i-q:i+q,j-q:j+q)); % apertura finestra in immagine bordi centrata su pixel scheletro
             SA_s = sins.*SA; % ricerca pixel edge sinistro
             temp_s = SA_s + mask_h;
             temp_s = temp_s == 2;
             ind_s = find(temp_s,1);
             if length(ind_s) < 1 % Salta se non trova bordi
                continue
             end
             [y_s,x_s] = ind2sub(size(SA_s),ind_s);
             u_s = [x_s-(q+1) y_s-(q+1)]; % vettore bordo sinistro
             SA_d = des.*SA; % ricerca pixel edge destro
             temp_d = SA_d + mask_h;
             temp_d = temp_d == 2;
             ind_d = find(temp_d,1);
             if length(ind_d) < 1 % Salta se non trova bordi
                continue
             end
             [y_d,x_d] = ind2sub(size(SA_d),ind_d);
             u_d = [x_d-(q+1) y_d-(q+1)]; % vettore bordo destro
             s(k).Y_skel = i-q;
             s(k).X_skel = j-q;
             s(k).X_edge_s = j - q - norm(u_s); % Riposiziona le coordinate nel sistema immagine corretto
             s(k).Y_edge_s= i - q;
             s(k).X_edge_d =j - q + norm(u_d);
             s(k).Y_edge_d= i - q;
             k = k+1;
          else  
             angler = -sign(angle)*(90 - abs(angle)); % direzione perpendicolare di ricerca, vale sia per angoli positivi che negativi
             SA = double(bord_edg(i-q:i+q,j-q:j+q)); % apertura finestra in immagine bordi centrata su pixel scheletro
             rot = imrotate(SA,angler,'bilinear','crop'); % Rotazione finestra tipo "bilineare"
             rot_s = sins.*rot; % ricerca pixel edge sinistro
             rot_s_r = imresize(rot_s, 2, 'bilinear'); % Resizing per avere distanze corrette
             if find( rot_s_r((2*q)+1,:)) == 0 % Salta se non trova bordi
                    continue
             end
             f_s = fit([1:size(rot_s_r,1)]' , rot_s_r((2*q)+1,:)' ,'gauss1', 'upper', [+Inf +Inf gauss_std]); % Fit gaussiana per trovare il punto più probabile del bordo lungo la riga centrale della finestra
             x_s = f_s.b1/2;
             y_s = q+1;
             u_s = [x_s-(q+1) y_s-(q+1)]; % vettore bordo sinistro
             rot_d = des.*rot; % ricerca pixel edge destro
             rot_d_r = imresize(rot_d, 2, 'bilinear'); % Resizing per avere distanze corrette
             if find( rot_d_r((2*q)+1,:)) == 0 % Salta se non trova bordi
                    continue
             end
             f_d = fit([1:size(rot_d_r,1)]' , rot_d_r((2*q)+1,:)' ,'gauss1', 'upper', [+Inf +Inf gauss_std]); % Fit gaussiana per trovare il punto più probabile del bordo lungo la riga centrale della finestra
             x_d = f_d.b1/2;
             y_d  = q+1;
             u_d = [x_d-(q+1) y_d-(q+1)]; % vettore bordo destro
             s(k).Y_skel = i-q;
             s(k).X_skel = j-q;
             s(k).X_edge_s = j - q + (cosd(angler)*u_s(1) - sind(angler)*u_s(2)); % Riposiziona le coordinate nel sistema immagine corretto
             s(k).Y_edge_s= i - q + (sind(angler)*u_s(1) + cosd(angler)*u_s(2));
             s(k).X_edge_d =j - q + (cosd(angler)*u_d(1) - sind(angler)*u_d(2));
             s(k).Y_edge_d= i - q + (sind(angler)*u_d(1) + cosd(angler)*u_d(2));
             k = k+1;
          end
       end
   end
end
%% salvataggio variabili

% Vengono salavate la struttura con associati scheletro-bordi e le immagini
% binarizzate di scheletro e bordi.

s0832 = s;
skel0832 = skel;
bw_edge0832 = bw_edge;
save('DJI_0832.mat', 's0832', 'skel0832', 'bw_edge0832')

%% Copia ciclo per ricavare immagini

sp = struct(); % Struttura per salvare coordinate immagine X e Y di scheletro, bordo destro e bordo sinistro
sp.X_skel = 0;
sp.Y_skel= 0;
sp.X_edge_s = 0;
sp.Y_edge_s= 0;
sp.X_edge_d = 0;
sp.Y_edge_d= 0;

for i = q+1:size(bord,1)-q
   for j = q+1:size(bord,2)-q
       if bord(i,j) == 1
%           i=354;
%           j=1875;
          IA = uint8(bord(i-p:i+p,j-p:j+p)); 
          [y,x] = find(IA);
          c = polyfit(x,y,1); % ricerca andamento lineare locale scheletro
          angle = atand(c(1));
          if angle >= -10 && angle <= 10 % Scheletro orizzontale
             SA = double(bord_edg(i-q:i+q,j-q:j+q));
             SA_s = su.*SA;
             temp_s= SA_s + mask_v;
             temp_s = temp_s == 2;
             ind_s = find(temp_s,1);
             if length(ind_s) < 1 % Salta se non trova bordi
                continue
             end
             [y_s,x_s] = ind2sub(size(SA_s),ind_s);
             u_s = [x_s-(q+1) y_s-(q+1)];
             SA_g = giu.*SA;
             temp_g = SA_g + mask_v;
             temp_g = temp_g == 2;
             ind_g = find(temp_g,1);
             if length(ind_g) < 1 % Salta se non trova bordi
                continue
             end
             [y_d,x_d] = ind2sub(size(SA_g),ind_g);
             u_d = [x_d-(q+1) y_d-(q+1)];
             sp(k).Y_skel = i-q;
             sp(k).X_skel = j-q;
             sp(k).X_edge_s = j - q; % Riposziona le coordinate nel sistema immagine corretto
             sp(k).Y_edge_s= i - q - norm(u_s);
             sp(k).X_edge_d = j - q;
             sp(k).Y_edge_d= i - q + norm(u_d);
             k = k+1;
          elseif angle <= -80 || angle >= 80 % Scheletro verticale
             SA = double(bord_edg(i-q:i+q,j-q:j+q));
             SA_s = sins.*SA;
             temp_s = SA_s + mask_h;
             temp_s = temp_s == 2;
             ind_s = find(temp_s,1);
             if length(ind_s) < 1 % Salta se non trova bordi
                continue
             end
             [y_s,x_s] = ind2sub(size(SA_s),ind_s);
             u_s = [x_s-(q+1) y_s-(q+1)];
             SA_d = des.*SA;
             temp_d = SA_d + mask_h;
             temp_d = temp_d == 2;
             ind_d = find(temp_d,1);
             if length(ind_d) < 1 % Salta se non trova bordi
                continue
             end
             [y_d,x_d] = ind2sub(size(SA_d),ind_d);
             u_d = [x_d-(q+1) y_d-(q+1)];
             sp(k).Y_skel = i-q;
             sp(k).X_skel = j-q;
             sp(k).X_edge_s = j - q - norm(u_s);% Riposziona le coordinate nel sistema immagine corretto
             sp(k).Y_edge_s= i - q;
             sp(k).X_edge_d =j - q + norm(u_d);
             sp(k).Y_edge_d= i - q;
             k = k+1;
          else 
             angler = -sign(angle)*(90 - abs(angle));
             SA = double(bord_edg(i-q:i+q,j-q:j+q));
%              SA2 = double(bord_edg(i-q:i+q,j-q:j+q));
%              imshowpair(SA1,SA2);
%              hold on;
%              plot(41,41, '+', 'Color', 'r', 'MarkerSize', 5)
             rot = imrotate(SA,angler,'bilinear','crop');
%              rot2 = imrotate(SA2,angle,'bilinear','crop');
%              imshowpair(rot1,rot2);
%              hold on;
%              plot(41,41, '+', 'Color', 'r', 'MarkerSize', 5)
             rot_s = sins.*rot;
             rot_s_r = imresize(rot_s, 2, 'bilinear');
             if find( rot_s_r((2*q)+1,:)) == 0
                    continue
             end
             f_s = fit([1:size(rot_s_r,1)]' , rot_s_r((2*q)+1,:)' ,'gauss1', 'upper', [+Inf +Inf gauss_std]);
%              plot(f_s, [1:size(rot_s_r,1)]', rot_s_r((2*q)+1,:)')
%              xlim([50 100])
%              title('Research of edge position')
%              xlabel('Pixel location', 'FontSize', 10, 'FontWeight', 'bold')
%              ylabel('Intensity', 'FontSize', 10, 'FontWeight', 'bold')
%              legend('FontWeight','bold')
%              f_s = fit([1:(2*q+1)]' , rot_s(q+1,:)' ,'gauss1', 'upper', [+Inf +Inf gauss_std]);
             x_s = f_s.b1/2;
             y_s = q+1;
             u_s = [x_s-(q+1) y_s-(q+1)];
             rot_d = des.*rot;
             rot_d_r = imresize(rot_d, 2, 'bilinear');
             if find( rot_d_r((2*q)+1,:)) == 0 % Salta se non trova bordi
                    continue
             end
             f_d = fit([1:size(rot_d_r,1)]' , rot_d_r((2*q)+1,:)' ,'gauss1', 'upper', [+Inf +Inf gauss_std]);
%              f_d = fit([1:(2*q+1)]' , rot_d(q+1,:)' ,'gauss1','upper', [+Inf +Inf gauss_std]);
             x_d = f_d.b1/2;
             y_d  = q+1;
             u_d = [x_d-(q+1) y_d-(q+1)];
             xs = cosd(angler)*u_s(1) - sind(angler)*u_s(2);
             ys = sind(angler)*u_s(1) + cosd(angler)*u_s(2);
             xd = cosd(angler)*u_d(1) - sind(angler)*u_d(2);
             yd = sind(angler)*u_d(1) + cosd(angler)*u_d(2);
             sp(k).Y_skel = i-q;
             sp(k).X_skel = j-q;
             sp(k).X_edge_s = j - q + xs;% Riposiziona le coordinate nel sistema immagine corretto
             sp(k).Y_edge_s = i - q + ys;
             sp(k).X_edge_d =j - q + xd;
             sp(k).Y_edge_d = i - q + yd;
             k = k+1;
          end
       end
   end
end

punti_s = undistortPoints([s.X_edge_s; s.Y_edge_s]', cameraParams);
punti_d = undistortPoints([s.X_edge_d; s.Y_edge_d]', cameraParams);
punti_sk = undistortPoints([s.X_skel; s.Y_skel]', cameraParams);



imshow(crepau)
hold on;
scatter(punti_s(:,1), punti_s(:,2), 0.5, 'g')
hold on;
scatter(punti_d(:,1), punti_d(:,2), 0.5, 'g')
hold on;
scatter(punti_sk(:,1), punti_sk(:,2), 0.5, 'r')