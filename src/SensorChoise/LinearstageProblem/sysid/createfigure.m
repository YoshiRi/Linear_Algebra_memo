function createfigure(XData1, YData1, XData2, YData2, XData3, YData3, XData4, YData4, YData5, YData6)
%CREATEFIGURE(XData1, YData1, XData2, YData2, XData3, YData3, XData4, YData4, YData5, YData6)
%  XDATA1:  line xdata
%  YDATA1:  line ydata
%  XDATA2:  line xdata
%  YDATA2:  line ydata
%  XDATA3:  line xdata
%  YDATA3:  line ydata
%  XDATA4:  line xdata
%  YDATA4:  line ydata
%  YDATA5:  line ydata
%  YDATA6:  line ydata

%  MATLAB ����̎���������: 27-Jan-2020 17:50:12

% figure ���쐬
figure1 = figure('PaperUnits','inches','PaperSize',[7.8 4.04166666666667]);

% axes ���쐬
axes1 = axes('Parent',figure1,...
    'Position',[0.146025641025641 0.485214948453608 0.758974358974359 0.388238659793814]);

% line ���쐬
line(XData1,YData1,'Visible','off','Parent',axes1);

% hggroup ���쐬
hggroup1 = hggroup('Parent',axes1,'DisplayName','Real Plant');

% line ���쐬
line(XData2,YData2,'Parent',hggroup1,'LineStyle','--',...
    'Color',[0 0.447 0.741]);

% hggroup ���쐬
hggroup2 = hggroup('Parent',axes1,'DisplayName','Fitted');

% line ���쐬
line(XData3,YData3,'Parent',hggroup2,'Color',[0.85 0.325 0.098]);

% ylabel ���쐬
ylabel('Gain (dB)','HitTest','off','FontSize',8);

% Axes �� X ���͈̔͂�ێ����邽�߂Ɉȉ��̃��C���̃R�����g������
% xlim(axes1,[2 80]);
% Axes �� Y ���͈̔͂�ێ����邽�߂Ɉȉ��̃��C���̃R�����g������
% ylim(axes1,[-70 0]);
box(axes1,'on');
% �c��̍��W���v���p�e�B�̐ݒ�
set(axes1,'FontSize',8,'XColor',[0 0 0],'XGrid','on','XMinorTick','on',...
    'XScale','log','XTickLabel','','YColor',[0 0 0],'YGrid','on');
% axes ���쐬
axes2 = axes('Parent',figure1,...
    'Position',[0.146025641025641 0.11 0.758974358974359 0.344287113402062]);

% line ���쐬
line(XData4,YData4,'Visible','off','Parent',axes2);

% hggroup ���쐬
hggroup3 = hggroup('Parent',axes2,'DisplayName','Real Plant');

% line ���쐬
line(XData2,YData5,'Parent',hggroup3,'LineStyle','--',...
    'Color',[0 0.447 0.741]);

% hggroup ���쐬
hggroup4 = hggroup('Parent',axes2,'DisplayName','Fitted');

% line ���쐬
line(XData3,YData6,'Parent',hggroup4,'Color',[0.85 0.325 0.098]);

% ylabel ���쐬
ylabel('Phase (deg)','HitTest','off','FontSize',8);

% Axes �� X ���͈̔͂�ێ����邽�߂Ɉȉ��̃��C���̃R�����g������
% xlim(axes2,[2 80]);
% Axes �� Y ���͈̔͂�ێ����邽�߂Ɉȉ��̃��C���̃R�����g������
% ylim(axes2,[-226.8 -43.2]);
box(axes2,'on');
% �c��̍��W���v���p�e�B�̐ݒ�
set(axes2,'FontSize',8,'XColor',[0 0 0],'XGrid','on','XMinorTick','on',...
    'XScale','log','YColor',[0 0 0],'YGrid','on','YTick',...
    [-225 -180 -135 -90 -45]);
% legend ���쐬
legend(axes2,'show');

% xlabel ���쐬
xlabel('Frequency  (rad/s)','HitTest','off','Units','pixels',...
    'HorizontalAlignment','center',...
    'FontSize',8,...
    'Visible','on');

% title ���쐬
title('Bode plot','HitTest','off','Units','pixels',...
    'HorizontalAlignment','center',...
    'FontSize',8);

