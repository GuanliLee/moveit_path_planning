# Raw MCAP is the collection system of record

The collection platform preserves the existing `episodeN` raw layout and MCAP topic contents and no longer generates legacy ALOHA HDF5, QC reports, or LeRobot datasets. Aligned HDF5, offline QC, repair, and LeRobot export belong to the independent quality pipeline; existing historical data is not rewritten or deleted by this decision.
