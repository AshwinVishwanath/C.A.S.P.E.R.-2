% Simple MATLAB test script to verify MATLAB is working in VS Code

% Test 1: Display a message
disp('MATLAB is working in VS Code!');

% Test 2: Basic arithmetic
result = 2 + 3;
fprintf('2 + 3 = %d\n', result);

% Test 3: Create a simple array and compute
arr = [1, 2, 3, 4, 5];
sum_arr = sum(arr);
fprintf('Sum of [1, 2, 3, 4, 5] = %d\n', sum_arr);

% Test 4: Create a simple plot
x = linspace(0, 2*pi, 100);
y = sin(x);
plot(x, y);
title('Simple Sine Wave');
xlabel('x');
ylabel('sin(x)');
grid on;

disp('All tests completed successfully!');