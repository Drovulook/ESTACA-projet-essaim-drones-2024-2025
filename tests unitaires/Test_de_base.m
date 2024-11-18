classdef Test_de_base < matlab.unittest.TestCase
    methods (Test)

        function test_shape(testCase)
        hold
        h_f = create_shape ('half_sphere','g',[0, 0, 0],[15, 15, 15],0,gca);
        cyl = create_shape ('cylinder','r',[0, 0, 0],[10, 10, 10],0,gca);
        box = create_shape ('box','y',[0, 0, 0],[5, 5, 5],0,gca);
        end

%entrée sortie 
   
    %shape = create_shape(shape_type, color, center, dimensions, angle, ax
    %shape = surf(ax, rotX, rotY, zc, 'FaceColor', color, 'EdgeColor', color, 'FaceAlpha', 0.3);
%%





%% exemple de nomenclature de test
       %function testPositiveNumbers(testCase)
            %actual = addition(2, 3);
            %expected = 5;
            %testCase.verifyEqual(actual, expected);
        %end
    end
end