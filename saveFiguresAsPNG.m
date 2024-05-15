function saveFiguresAsPNG()

    % Get the current MATLAB script file's directory
    current_directory = fileparts(mfilename('fullpath'));

    % Create the 'Figures' subfolder if it doesn't exist
    figures_directory = fullfile(current_directory, 'Figures');
    if ~exist(figures_directory, 'dir')
        mkdir(figures_directory);
    end

    % Loop through open figures and save them as PNG files
    figHandles = findobj('type', 'figure');
    for i = 1:length(figHandles)
        fig = figure(i);

        % Set the font to Times New Roman
        set(findall(fig, '-property', 'FontName'), 'FontName', 'Times New Roman');

        % Set the font size and line width in points
        font_size = 12; % Change this to your desired font size in points
        line_width = 1.5; % Change this to your desired line width in points

        set(findall(fig, '-property', 'FontSize'), 'FontSize', font_size);
        set(findall(fig, '-property', 'LineWidth'), 'LineWidth', line_width);

        % Remove unnecessary white space by adjusting the axes
        set(gca, 'LooseInset', get(gca, 'TightInset'));

        % Get the plot title
        plot_title = get(get(gca, 'Title'), 'String');

        % Use the plot title as the filename if available, otherwise use the figure number
        if isempty(plot_title)
            filename = sprintf('figure%d.png', i);
        else
            % Remove any characters that are not suitable for filenames
            plot_title = regexprep(plot_title, '[^a-zA-Z0-9-_]', '_');
            filename = [plot_title, '.png'];
        end

        % Save the figure in the 'Figures' subfolder
        full_filename = fullfile(figures_directory, filename);
        print(fig, full_filename, '-dpng', '-r300'); % Adjust resolution as needed
    end

    close all
    fprintf('All done\n')
end
