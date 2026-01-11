import pandas as pd
import re

def extract_numeric_values(text):
    """Extract all numeric values from a text string"""
    return re.findall(r'-?\d+\.?\d*', text)

def clean_bms_data_comprehensive(input_file, output_file):
    """
    Comprehensive cleaning of BMS data
    """
    # Read the raw file line by line
    with open(input_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    all_data = []
    
    # Process each line
    for line_num, line in enumerate(lines, 1):
        line = line.strip()
        if not line:
            continue
            
        # Extract all numeric values from the line
        values = extract_numeric_values(line)
        
        # Debug: Check first few lines
        if line_num <= 3:
            print(f"Line {line_num}: Found {len(values)} values")
            print(f"Values: {values[:10]}...")
        
        all_data.append(values)
    
    # Find the maximum number of columns
    max_cols = max(len(row) for row in all_data if row)
    print(f"\nMaximum columns found: {max_cols}")
    
    # Create headers based on pattern
    headers = ['Readings']
    
    # Based on your data pattern:
    # 1. Readings
    # 2. Voltage(V)
    # 3. Current(ma)
    # 4. SoC(%)
    # 5. Discharge Rate(W)
    # 6. Energy(mWh)
    # 7. Charge(mAh)
    # 8-19. Cell 1-12
    # 20-31. Temp 1-12
    
    if max_cols == 31:  # Your expected format
        headers.extend([
            'Voltage(V)', 'Current(ma)', 'SoC(%)', 
            'Discharge Rate(W)', 'Energy(mWh)', 'Charge(mAh)',
            'Cell1(mV)', 'Cell2(mV)', 'Cell3(mV)', 'Cell4(mV)', 
            'Cell5(mV)', 'Cell6(mV)', 'Cell7(mV)', 'Cell8(mV)',
            'Cell9(mV)', 'Cell10(mV)', 'Cell11(mV)', 'Cell12(mV)',
            'Temp1(°C)', 'Temp2(°C)', 'Temp3(°C)', 'Temp4(°C)',
            'Temp5(°C)', 'Temp6(°C)', 'Temp7(°C)', 'Temp8(°C)',
            'Temp9(°C)', 'Temp10(°C)', 'Temp11(°C)', 'Temp12(°C)'
        ])
    else:
        # Generic headers based on actual column count
        headers = [f'Column_{i+1}' for i in range(max_cols)]
        print(f"Using generic headers: {headers}")
    
    # Create DataFrame
    df = pd.DataFrame(all_data, columns=headers)
    
    # Convert all columns to numeric
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    
    # Save to CSV
    df.to_csv(output_file, index=False)
    
    print(f"\nData Summary:")
    print(f"Total rows: {len(df)}")
    print(f"Total columns: {len(df.columns)}")
    print(f"Data saved to: {output_file}")
    
    return df

# Simpler approach using split instead of regex
def clean_bms_data_simple_split(input_file, output_file):
    """
    Simple approach using split on commas
    """
    all_data = []
    
    with open(input_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            
            # Split by comma
            parts = line.split(',')
            row = []
            
            for part in parts:
                # Extract just the numeric value from each part
                # Format: "Label:value" or "Label value"
                if ':' in part:
                    # Format like "Readings:1" or "Cell 1:4002"
                    value_part = part.split(':')[-1]
                else:
                    value_part = part
                
                # Remove any non-numeric characters except minus and dot
                value_part = ''.join(c for c in value_part if c.isdigit() or c == '-' or c == '.')
                
                if value_part:  # Only add if we found a value
                    try:
                        # Try to convert to float first (for potential decimal values)
                        num = float(value_part)
                        # Convert to int if it's a whole number
                        if num.is_integer():
                            row.append(int(num))
                        else:
                            row.append(num)
                    except ValueError:
                        # If conversion fails, keep as string
                        row.append(value_part)
            
            all_data.append(row)
    
    # Find max columns
    max_cols = max(len(row) for row in all_data)
    print(f"Maximum columns: {max_cols}")
    
    # Create headers
    if max_cols == 31:
        headers = [
            'Readings', 'Voltage(V)', 'Current(ma)', 'SoC(%)', 
            'Discharge Rate(W)', 'Energy(mWh)', 'Charge(mAh)',
            'Cell1(mV)', 'Cell2(mV)', 'Cell3(mV)', 'Cell4(mV)', 
            'Cell5(mV)', 'Cell6(mV)', 'Cell7(mV)', 'Cell8(mV)',
            'Cell9(mV)', 'Cell10(mV)', 'Cell11(mV)', 'Cell12(mV)',
            'Temp1(°C)', 'Temp2(°C)', 'Temp3(°C)', 'Temp4(°C)',
            'Temp5(°C)', 'Temp6(°C)', 'Temp7(°C)', 'Temp8(°C)',
            'Temp9(°C)', 'Temp10(°C)', 'Temp11(°C)', 'Temp12(°C)'
        ]
    else:
        headers = [f'Col{i+1}' for i in range(max_cols)]
    
    df = pd.DataFrame(all_data, columns=headers)
    df.to_csv(output_file, index=False)
    
    print(f"\nData saved to {output_file}")
    print(f"Shape: {df.shape}")
    print(f"\nFirst few rows:")
    print(df.head())
    
    return df

# Quick and dirty approach - just remove text prefixes
def quick_clean(input_file, output_file):
    """
    Fastest approach - replace all text prefixes
    """
    import csv
    
    with open(input_file, 'r', encoding='utf-8') as infile, \
         open(output_file, 'w', newline='', encoding='utf-8') as outfile:
        
        writer = csv.writer(outfile)
        
        # Write header
        header = [
            'Readings', 'Voltage(V)', 'Current(ma)', 'SoC(%)', 
            'Discharge Rate(W)', 'Energy(mWh)', 'Charge(mAh)',
            'Cell1(mV)', 'Cell2(mV)', 'Cell3(mV)', 'Cell4(mV)', 
            'Cell5(mV)', 'Cell6(mV)', 'Cell7(mV)', 'Cell8(mV)',
            'Cell9(mV)', 'Cell10(mV)', 'Cell11(mV)', 'Cell12(mV)',
            'Temp1(°C)', 'Temp2(°C)', 'Temp3(°C)', 'Temp4(°C)',
            'Temp5(°C)', 'Temp6(°C)', 'Temp7(°C)', 'Temp8(°C)',
            'Temp9(°C)', 'Temp10(°C)', 'Temp11(°C)', 'Temp12(°C)'
        ]
        writer.writerow(header)
        
        # Process each line
        for line in infile:
            line = line.strip()
            if not line:
                continue
            
            # Remove all text before colons
            # Pattern: match any text followed by colon and capture the number
            import re
            numbers = re.findall(r':(-?\d+\.?\d*)', line)
            
            if numbers:
                # Convert to appropriate numeric types
                row = []
                for num in numbers:
                    try:
                        if '.' in num:
                            row.append(float(num))
                        else:
                            row.append(int(num))
                    except:
                        row.append(num)
                
                writer.writerow(row)

# Run the cleaner
if __name__ == "__main__":
    input_file = "BMS Testing Main.csv"
    output_file = "BMS_CLEANED_MAIN.csv"
    
    print("Cleaning BMS data...")
    
    # Try method 1 first
    try:
        df = clean_bms_data_comprehensive(input_file, output_file)
        print("\n✓ Method 1 completed successfully!")
    except Exception as e:
        print(f"\n✗ Method 1 failed: {e}")
        
        # Try method 2
        try:
            print("\nTrying method 2...")
            df = clean_bms_data_simple_split(input_file, output_file)
            print("\n✓ Method 2 completed successfully!")
        except Exception as e2:
            print(f"\n✗ Method 2 failed: {e2}")
            
            # Try method 3
            try:
                print("\nTrying method 3 (quick clean)...")
                quick_clean(input_file, output_file)
                print("\n✓ Method 3 completed successfully!")
            except Exception as e3:
                print(f"\n✗ All methods failed: {e3}")
                print("\nPlease check your input file format.")