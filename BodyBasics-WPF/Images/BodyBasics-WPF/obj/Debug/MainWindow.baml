   M S B A M L   `   `   `  ����  BodyBasics-WPF4    .Microsoft.Samples.Kinect.BodyBasics.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   -   5          � Title$  JustRICE��5   	    � WindowState$ 	Maximized=�5   	   $	��600��5   	   $	��700��6    � 	Resources 5	         MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush��%�   &	        &	 #     &	 F     �� 5
   
   $ 	#ff6e6e6e��62   �� 5   
   $ 	#ff52318f��64   �� 5   
   $ 	#ff00BCF2��62   5      .��5      � $��	10 0 10 0q�6    � RowDefinitions 5   
   �� 5      $
��Auto �6   �� 5      $��* �6   �� 5      $
��Auto �6    � ColumnDefinitions 5   
   �� 5      $��50 �6   �� 5      $	��500 �6   �� 5      $��* �6   �� 5      $��50 �6   .+�5   
   �� $��0��6   $��0��6   $��Images\mmlogo.png��6-   $��Center=�6H   $��Center=�6e   $
��Fill=�6�   $��32��6�   $��32��6�   $��0 10 0 5q�6�   �� 5   
   $��0��6   $��1��6!   $��0 0 -1 0q�61   $
��Left=�6C   $��Center=�6^   #��[  6y   $��Segoe UI)�6�   $��18(�6�   Move By Move6�   (� 5!   
   $��1��6   $��0��6   $
��Left=�6/   $	��200��6J   $	��400��6Z   .��5"      �� -   5         ��Name$ 	LeftArrow��5"      $��Center=�6(   #��[ 6C   $��Segoe UI)�6�   $��16(�6�   $��40��6�   $��13��6�   .��6�   <6�   5#      �� 5%   
   -   5         ��Name$ Display��5%      $��1��6!   $��1��6.   $��Center=�6>   $��Center=�6[   $��Images\1.jpg��6v   (� 5*   
   $��1��6   $��2��6   $��Center=�6/   .��5+      �� $��UniformToFill=�63   ��6   �� *ImageSource+63   5,      (� 5-   
   $��1��6   $��3��6   $��Right=�6/   $	��200��6K   $	��400��6[   .��5.      �� -   5        $ 
RightArrow��5.      $��Center=�6)   #��[ 6D   $��Segoe UI)�6�   $��16(�6�   $��45��6�   $��13��6�   .��6�   >6�   5/      �� 51   
   -   5        $��	statusBar��51   ?   $��2��6   $��Stretch=�6!   $��Bottom=�6P   
�������#��[  6~   .@�52      �� ��6   �� *
StatusText+53      54      55      