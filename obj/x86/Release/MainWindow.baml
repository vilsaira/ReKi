   M S B A M L   `   `   `  ���� @  <ReKi2, Version=1.0.0.0, Culture=neutral, PublicKeyToken=nullclr-namespace:ReKi2ReKi2      ReKi2.MainWindow   W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     @d2http://schemas.microsoft.com/expression/blend/2008     Jmc;http://schemas.openxmlformats.org/markup-compatibility/2006     %localclr-namespace:ReKi2     -     � Title$  Record Kinect 2.0��$	��700��$
��1500�� � 	Resources    
BlackBrush �� Color  MediumGreyBrush �� 
TargetType �� Property �� Value��%y   &	        &	      '�� &     �� $ Black���� $ 	#ff6e6e6e���� " ��.���  �@J�SnapsToDevicePixels . .��� $��	10 0 10 0q� � RowDefinitions �� $
��Auto ��� $��* ��� $
��Auto ��� $
��Auto � � ColumnDefinitions �� $
��Auto ��� $
��Auto ��� $
��Auto �.+��� -    ��Name$ txtb_colorTime��$��0��$��1��$
��Left=�$	��Top=�$��0q�$��5 0 0 0q�#��[  $��Segoe UI)�$��12(�.��Color time:�� -   $ txtb_depthTime��$��0��$��1��$
��Left=�$	��Top=�$��0q�$��5 20 0 0q�#��[  $��Segoe UI)�$��12(�.��Depth time:�� -   $ txtb_distance��$��0��$��1��$
��Left=�$	��Top=�$��0q�$��5 40 0 0q�#��[  $��Segoe UI)�$��12(�.��
Distance�� -   $ txtb_Author��$��3��$��2��$��Right=�$��Segoe UI)�$��12(�$��0 0 2 3q�.��=;Developed by Viljami Sairanen (2016-2017) www.babacenter.fi(� $��1��$��2��$��Center=�$��3,3,3,3q�$	��450��.���� -   	 ��Name$	 cameraColor��$��UniformToFill=�(� $��1��$��1��$��Center=�$��3,3,3,3q�$	��450��.���� -   $��cameraDepth��$��UniformToFill=�� $��1��  GridButtonStyle#��[ .+���   StackPanelButtonStyle#��[ .+��� -   $ txtb_stopwatch��$
��Left=�$	��Top=�$��0 0 0 20q�$��3q�#��[  $��Segoe UI)�$��25(�.��
00:00:00�� -	   
 ��Name$
 btn_RecordPause��$��Record��  ButtonStyle#��[ $��0 2 0 2q�
�������� -
   $
 btn_Convert��$��Convert��#��[ $��0 2 0 25q��� -   $��	statusBar��$��2��$��Stretch=�$��Bottom=�
������ #��[ .@��� ���� *
StatusText+� -    �Name$ recordingText��$��4��$��3��$	��300��$��40��$��Center=�$��Center=�$��3,3,3,3q�$y�	Collapsed=�
���   � J� Opacity$
 0.75Q�.+��� .+��� $��Recording...��$
��Bold"�$��Cambria)�$��Bottom=�$��Center=�
�������$��36(�� -   $ 
Processing��$��4��$��3��$	��300��$��40��$��Center=�$��Center=�$��3,3,3,3q�$y�	Collapsed=�
���   �$
 0.75Q�.+��� .+��� $��
Loading...��$
��Bold"�$��Cambria)�$��Bottom=�$��Center=�
�������$��36(�� -   $ InputBox��$��2��$��3��$y�	Collapsed=�.+�� 
���   �$	 0.5Q��� $	��250��
����ح�
���   �$��1q� �� CornerRadius$ 	55,0,55,0��$��Center=�$��Center=�._��� .+��� $��5q�$��Subject ID:��$
��Bold"�$��Cambria)�$��Center=�
���   ��� -   $ txtb_SubjectID��$	��150��$��Center=�$��Center=���  �� Orientation$ 
Horizontal=�$��Center=�.+��� -   $
 btn_Continue��$��5q�$��Continue����Z� �� -   $
 
btn_Cancel��$��5q�$��Cancel����Z� 