
def convert_columns_dtype(df, old_dtype, new_dtype):
    """
    Parameters
    ----------
    df: pandas.DataFrame

    old_dtype: numpy dtype

    new_dtype: numpy dtype
    """
    changed = []
    for column in df.columns:
        if df[column].dtype == old_dtype:
            df[column] = df[column].astype(new_dtype)
            changed.append(column)

    return changed
